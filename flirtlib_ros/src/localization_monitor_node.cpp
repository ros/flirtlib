/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * \file 
 * 
 * Node that tracks how good our localization is
 *
 * \author Bhaskara Marthi
 */

#include <flirtlib_ros/localization_monitor.h>
#include <flirtlib_ros/conversions.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <mongo_ros/message_collection.h>
#include <visualization_msgs/Marker.h>

namespace flirtlib_ros
{

namespace sm=sensor_msgs;
namespace gm=geometry_msgs;
namespace nm=nav_msgs;
namespace mr=mongo_ros;
namespace vm=visualization_msgs;

using std::string;
using std::vector;
using boost::format;
using boost::shared_ptr;

typedef boost::mutex::scoped_lock Lock;
typedef mr::MessageWithMetadata<RefScanRos>::ConstPtr DBScan;
typedef vector<InterestPoint*> InterestPointVec;

class Node
{
public:

  Node ();

  void scanCB (sm::LaserScan::ConstPtr scan);
  
  void mapCB (const nm::OccupancyGrid& g);
  
private:
  
  gm::Pose getPose ();
  RefScanRos extractFeatures (sm::LaserScan::ConstPtr scan,
                              const gm::Pose& pose) const;

  void updateLocalized (sm::LaserScan::ConstPtr scan);
  void updateUnlocalized ();
  
  // Needed during init
  boost::mutex mutex_;
  ros::NodeHandle nh_;
  
  // Flirtlib objects
  shared_ptr<SimpleMinMaxPeakFinder> peak_finder_;
  shared_ptr<HistogramDistance<double> > histogram_dist_;
  shared_ptr<Detector> detector_;
  shared_ptr<DescriptorGenerator> descriptor_;
  shared_ptr<RansacFeatureSetMatcher> ransac_;

  shared_ptr<ScanPoseEvaluator> evaluator_;
  bool previously_matched_;

  mr::MessageCollection<RefScanRos> scans_;
  tf::TransformListener tf_;
  ros::Subscriber scan_sub_;
  ros::Subscriber map_sub_;
  ros::Publisher marker_pub_;
};


Detector* createDetector (SimpleMinMaxPeakFinder* peak_finder)
{
  const double scale = 5.0;
  const double dmst = 2.0;
  const double base_sigma = 0.2;
  const double sigma_step = 1.4;
  CurvatureDetector* det = new CurvatureDetector(peak_finder, scale, base_sigma,
                                                 sigma_step, dmst);
  det->setUseMaxRange(false);
  return det;
}

DescriptorGenerator* createDescriptor (HistogramDistance<double>* dist)
{
  const double min_rho = 0.02;
  const double max_rho = 0.5;
  const double bin_rho = 4;
  const double bin_phi = 12;
  BetaGridGenerator* gen = new BetaGridGenerator(min_rho, max_rho, bin_rho,
                                                 bin_phi);
  gen->setDistanceFunction(dist);
  return gen;
}

// Constructor sets up ros comms and flirtlib objects
Node::Node () :
  peak_finder_(new SimpleMinMaxPeakFinder(0.34, 0.001)),
  histogram_dist_(new SymmetricChi2Distance<double>()),
  detector_(createDetector(peak_finder_.get())),
  descriptor_(createDescriptor(histogram_dist_.get())),
  ransac_(new RansacFeatureSetMatcher(0.0599, 0.95, 0.4, 0.4,
                                      0.0384, false)),
  previously_matched_(false),
  scans_("flirtlib_place_rec", "scans", "mongo.willowgarage.com"),
  scan_sub_(nh_.subscribe("base_scan", 10, &Node::scanCB, this)),
  map_sub_(nh_.subscribe("map", 10, &Node::mapCB, this)),
  marker_pub_(nh_.advertise<vm::Marker>("visualization_marker", 10))
{
  ROS_INFO("Localization monitor initialized");
}


void Node::mapCB (const nm::OccupancyGrid& g)
{
  Lock l(mutex_);
  ROS_INFO("Received map; setting scan evaluator");
  evaluator_.reset(new ScanPoseEvaluator(g));
  ROS_INFO("Scan evaluator initialized");
}

inline
gm::Pose toMsg (const tf::Transform& trans)
{
  gm::Pose m;
  tf::poseTFToMsg(trans, m);
  return m;
}

gm::Pose Node::getPose ()
{
  tf::StampedTransform trans;
  tf_.lookupTransform("/map", "base_laser_link", ros::Time(), trans);
  gm::Pose pose;
  tf::poseTFToMsg(trans, pose);
  return pose;
}

RefScanRos Node::extractFeatures (sm::LaserScan::ConstPtr scan,
                                  const gm::Pose& pose) const
{
  shared_ptr<LaserReading> reading = fromRos(*scan);
  InterestPointVec pts;
  detector_->detect(*reading, pts);
  BOOST_FOREACH (InterestPoint* p, pts) 
    p->setDescriptor(descriptor_->describe(*p, *reading));
  marker_pub_.publish(interestPointMarkers(pts, pose));
  const RefScan ref(scan, pose, pts);
  return toRos(ref);
}



void Node::updateUnlocalized ()
{
    ROS_INFO("Not well localized");
    if (previously_matched_)
      ROS_INFO("Previously matched, so not doing anything");
    else
    {
      ROS_INFO("Here's where I'd try to match this scan");
    }
}

void Node::updateLocalized (sm::LaserScan::ConstPtr scan)
{
    ROS_INFO("Well localized");
    const double DPOS = 0.5;
    const double DTHETA = 0.5;
  
    gm::Pose current = getPose();
    const double x = current.position.x;
    const double y = current.position.y;
    const double th = tf::getYaw(current.orientation);
    format f("{x : {$gt: %.4f, $lt: %.4f}, y : {$gt: %.4f, $lt: %.4f}, "
             "theta: {$gt: %.4f, $lt: %.4f} }");
    string str = (f % (x-DPOS) % (x+DPOS) % (y-DPOS) % (y+DPOS) %
                  (th-DTHETA) % (th+DTHETA)).str();
    mongo::Query q = mongo::fromjson(str);
    
    vector<DBScan> scans = scans_.pullAllResults(q, true);
    ROS_INFO ("Found %zu nearby scans", scans.size());
    if (scans.size()<1)
    {
      scans_.insert(extractFeatures(scan, current),
                    mr::Metadata("x", x, "y", y, "theta", th));
      ROS_DEBUG_NAMED ("save_scan", "Saved scan at %.2f, %.2f, %.2f", x, y, th);
    }
}


// Given current scan and pose, evaluate how good our localization is
void Node::scanCB (sm::LaserScan::ConstPtr scan)
{
  // First, look up the sensor pose at the time of the scan
  tf_.waitForTransform("/map", scan->header.frame_id, scan->header.stamp,
                       ros::Duration(0.5));
  tf::StampedTransform trans;
  try
  {
    tf_.lookupTransform("/map", scan->header.frame_id, scan->header.stamp,
                        trans);
  }
  catch (tf::TransformException& e)
  {
    ROS_INFO_STREAM ("Skipping scan due to tf exception " << e.what());
    return;
  }
  
  // Have we initialized the evaluator yet?
  Lock l(mutex_);
  if (!evaluator_)
  {
    ROS_INFO_STREAM ("Skipping scan as evaluator not yet initialized");
    return;
  }
    
  // Evaluate how well this scan is localized
  const double dist = (*evaluator_)(*scan, toMsg(trans));
  //ROS_INFO_THROTTLE (1.0, "Median distance is %.4f", dist);
  
  if (dist < 0.1)
    updateLocalized(scan);
  else
    updateUnlocalized();
}

} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "localization_monitor_node");
  flirtlib_ros::Node node;
  ros::spin();
  return 0;
}
