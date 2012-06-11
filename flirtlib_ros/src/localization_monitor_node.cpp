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
#include <flirtlib_ros/common.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mongo_ros/message_collection.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseActionResult.h>

namespace flirtlib_ros
{

namespace sm=sensor_msgs;
namespace gm=geometry_msgs;
namespace nm=nav_msgs;
namespace mr=mongo_ros;
namespace vm=visualization_msgs;
namespace mbm=move_base_msgs;

using std::string;
using std::vector;
using boost::format;
using boost::shared_ptr;

typedef boost::mutex::scoped_lock Lock;
typedef mr::MessageWithMetadata<RefScanRos>::ConstPtr DBScan;
typedef vector<InterestPoint*> InterestPointVec;
typedef std::pair<InterestPoint*, InterestPoint*> Correspondence;
typedef vector<Correspondence> Correspondences;
typedef vector<RefScan> RefScans;

class Node
{
public:

  Node ();

  void scanCB (sm::LaserScan::ConstPtr scan);
  
  void mapCB (const nm::OccupancyGrid& g);
  
  void navCB (const mbm::MoveBaseActionResult& m);
  
private:
  
  gm::Pose getPose ();
  InterestPointVec extractFeatures (sm::LaserScan::ConstPtr scan) const;

  void updateLocalized (sm::LaserScan::ConstPtr scan, const gm::Pose& p);
  void updateUnlocalized (sm::LaserScan::ConstPtr scan);
  void publishRefScans () const;
  
  // Needed during init
  boost::mutex mutex_;
  ros::NodeHandle nh_;
  
  // Parameters
  unsigned min_num_matches_;
  unsigned min_successful_navs_;
  string db_name_;
  
  // Flirtlib objects
  shared_ptr<SimpleMinMaxPeakFinder> peak_finder_;
  shared_ptr<HistogramDistance<double> > histogram_dist_;
  shared_ptr<Detector> detector_;
  shared_ptr<DescriptorGenerator> descriptor_;
  shared_ptr<RansacFeatureSetMatcher> ransac_;

  shared_ptr<ScanPoseEvaluator> evaluator_;
  vector<RefScan> ref_scans_; // local copy of db scans
  tf::Transform laser_offset_;
  unsigned match_counter_;
  unsigned successful_navs_;

  mr::MessageCollection<RefScanRos> scans_;
  tf::TransformListener tf_;
  ros::Subscriber scan_sub_;
  ros::Subscriber map_sub_;
  ros::Publisher marker_pub_;
  ros::Publisher ref_scan_pose_pub_;
  ros::Publisher pose_est_pub_;
  ros::Publisher match_pose_pub_;
  ros::Publisher initial_pose_pub_;
  ros::Subscriber nav_result_sub_;
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
  min_num_matches_(getPrivateParam<int>("min_num_matches", 8)),
  min_successful_navs_(getPrivateParam<int>("min_successful_navs", 0)),
  db_name_(getPrivateParam<string>("db_name", "flirtlib_place_rec")),
  peak_finder_(new SimpleMinMaxPeakFinder(0.34, 0.001)),
  histogram_dist_(new SymmetricChi2Distance<double>()),
  detector_(createDetector(peak_finder_.get())),
  descriptor_(createDescriptor(histogram_dist_.get())),
  ransac_(new RansacFeatureSetMatcher(0.0599, 0.95, 0.4, 0.4,
                                      0.0384, false)),
  match_counter_(0), successful_navs_(0),
  scans_(db_name_, "scans", "mongo.willowgarage.com"),
  scan_sub_(nh_.subscribe("base_scan", 10, &Node::scanCB, this)),
  map_sub_(nh_.subscribe("map", 10, &Node::mapCB, this)),
  marker_pub_(nh_.advertise<vm::Marker>("visualization_marker", 10)),
  ref_scan_pose_pub_(nh_.advertise<gm::PoseArray>("ref_scan_poses", 10, true)),
  pose_est_pub_(nh_.advertise<gm::PoseStamped>("pose_estimate", 1)),
  match_pose_pub_(nh_.advertise<gm::PoseArray>("match_poses", 1)),
  initial_pose_pub_(nh_.advertise<gm::PoseWithCovarianceStamped>("initialpose", 1)),
  nav_result_sub_(nh_.subscribe("move_base/result", 10, &Node::navCB, this))
{
  ROS_DEBUG_NAMED("init", "Waiting for laser offset");
  laser_offset_ = getLaserOffset(tf_);

  BOOST_FOREACH (const mr::MessageWithMetadata<RefScanRos>::ConstPtr m,
                 scans_.queryResults(mr::Query(), false)) 
    ref_scans_.push_back(fromRos(*m));
  publishRefScans();
  ROS_INFO("Localization monitor initialized with %zu scans", ref_scans_.size());
}

void Node::navCB (const mbm::MoveBaseActionResult& res)
{
  if (res.status.status == actionlib_msgs::GoalStatus::SUCCEEDED)
  {
    successful_navs_++;
    if (successful_navs_==min_successful_navs_)
    {
      ROS_INFO ("Have now observed %u successful navigations.  Can "
                "now start saving scans", successful_navs_);
    }
  }
}

void Node::publishRefScans () const
{
  gm::PoseArray poses;
  poses.header.stamp = ros::Time::now();
  poses.header.frame_id = "/map";
  BOOST_FOREACH (const RefScan& s, ref_scans_) 
    poses.poses.push_back(s.pose);
  ref_scan_pose_pub_.publish(poses);
}


void Node::mapCB (const nm::OccupancyGrid& g)
{
  Lock l(mutex_);
  ROS_INFO("Received map; setting scan evaluator");
  evaluator_.reset(new ScanPoseEvaluator(g));
  ROS_INFO("Scan evaluator initialized");
}

InterestPointVec Node::extractFeatures (sm::LaserScan::ConstPtr scan) const
{
  shared_ptr<LaserReading> reading = fromRos(*scan);
  InterestPointVec pts;
  detector_->detect(*reading, pts);
  BOOST_FOREACH (InterestPoint* p, pts) 
    p->setDescriptor(descriptor_->describe(*p, *reading));
  return pts;
}



void Node::updateUnlocalized (sm::LaserScan::ConstPtr scan)
{
  gm::Pose current = getCurrentPose(tf_, "base_footprint");
  ROS_INFO("Not well localized");
  InterestPointVec pts = extractFeatures(scan);
  marker_pub_.publish(interestPointMarkers(pts, current));
      
  gm::PoseArray match_poses;
  match_poses.header.frame_id = "/map";
  match_poses.header.stamp = ros::Time::now();
  int best_num_matches = -1;
  gm::Pose best_pose;
  
  BOOST_FOREACH (const RefScan& ref_scan, ref_scans_) 
  {
    ROS_INFO ("Matching scan");
    Correspondences matches;
    OrientedPoint2D trans;
    ransac_->matchSets(ref_scan.raw_pts, pts, trans, matches);
    const int num_matches = matches.size();
    if (num_matches > static_cast<int>(min_num_matches_)) 
    {
      ROS_INFO_NAMED ("match", "Found %d matches with ref scan at "
                      "%.2f, %.2f, %.2f", num_matches,
                      ref_scan.pose.position.x, ref_scan.pose.position.y,
                      tf::getYaw(ref_scan.pose.orientation));
      match_poses.poses.push_back(ref_scan.pose);
      const gm::Pose laser_pose = transformPose(ref_scan.pose, trans);
      //const tf::Transform temp(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
      if (num_matches > best_num_matches)
      {
        best_num_matches = num_matches;
        best_pose = laser_pose;
      }
    }
  }

  if (best_num_matches<static_cast<int>(min_num_matches_))
    return;

  match_pose_pub_.publish(match_poses);
  const double quality = (*evaluator_)(*scan, best_pose);
  ROS_INFO ("Quality is %.2f", quality);
  if (quality < 0.2)
  {
    ROS_INFO("Found a good match");
    
    gm::PoseStamped estimated_pose;
    tf::poseTFToMsg(poseMsgToTf(best_pose)*laser_offset_, estimated_pose.pose);
    estimated_pose.header.frame_id = "/map";
    estimated_pose.header.stamp = ros::Time::now();
    pose_est_pub_.publish(estimated_pose);
    
    // Only publish initialpose once
    if (match_counter_==0)
    {
      match_counter_++;
      gm::PoseWithCovarianceStamped initial_pose;
      initial_pose.header.frame_id = "/map";
      initial_pose.header.stamp = ros::Time::now();
      initial_pose.pose.pose = estimated_pose.pose;
      initial_pose_pub_.publish(initial_pose);
    }
  }
}

void Node::updateLocalized (sm::LaserScan::ConstPtr scan,
                            const gm::Pose& current)
{
    ROS_INFO("Well localized");
    const double DPOS = 0.5;
    const double DTHETA = 0.5;
  
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
    if (scans.size()<1 && successful_navs_>=min_successful_navs_) 
    {
      InterestPointVec pts = extractFeatures(scan);
      RefScan ref_scan(scan, current, pts);
      ref_scans_.push_back(ref_scan);
      scans_.insert(toRos(ref_scan),
                    mr::Metadata("x", x, "y", y, "theta", th));
      ROS_DEBUG_NAMED ("save_scan", "Saved scan at %.2f, %.2f, %.2f", x, y, th);
      publishRefScans();
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
  const gm::Pose pose = tfTransformToPose(trans);
  const double dist = (*evaluator_)(*scan, pose);
  ROS_INFO_THROTTLE (1.0, "Scan quality is %.2f", dist);
  
  if (dist < 0.15)
    updateLocalized(scan, pose);
  else
    updateUnlocalized(scan);
}

} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "localization_monitor_node");
  flirtlib_ros::Node node;
  ros::spin();
  return 0;
}
