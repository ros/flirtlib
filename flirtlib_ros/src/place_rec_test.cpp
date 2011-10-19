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
 * Loads occupancy grid from file.  Then listens to scans and visualizes top
 * place recognition matches using flirtlib
 *
 * \author Bhaskara Marthi
 */

#define BOOST_NO_HASH
#include <flirtlib_ros/flirtlib.h>

#include <flirtlib_ros/conversions.h>
#include <occupancy_grid_utils/file.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>

namespace sm=sensor_msgs;
namespace gu=occupancy_grid_utils;
namespace vm=visualization_msgs;
namespace gm=geometry_msgs;
namespace nm=nav_msgs;

using std::string;
using std::vector;

typedef boost::mutex::scoped_lock Lock;
typedef vector<InterestPoint*> InterestPointVec;
typedef std::pair<InterestPoint*, InterestPoint*> Correspondence;
typedef vector<Correspondence> Correspondences;

namespace flirtlib_ros
{

/************************************************************
 * Utilities
 ***********************************************************/

struct RefScan
{
  sm::LaserScan::ConstPtr scan;
  gm::Pose pose;
  InterestPointVec pts;

  RefScan (sm::LaserScan::ConstPtr scan, const gm::Pose& pose,
           const InterestPointVec& pts) :
    scan(scan), pose(pose), pts(pts)
  {}
};

/************************************************************
 * Node class
 ***********************************************************/

class Node
{
public:

  Node ();
  void mainLoop (const ros::TimerEvent& e);
  void scanCB (sm::LaserScan::ConstPtr scan);

private:

  void initializeRefScans();
  gm::Pose getPose();

  // Needed during initialization
  boost::mutex mutex_;
  ros::NodeHandle nh_;

  // Parameters 
  const string map_file_;
  const double resolution_;
  const sm::LaserScan scanner_params_;
  const double ref_pos_resolution_;
  const double ref_angle_resolution_;
  const double x0_, x1_, y0_, y1_;
  const unsigned num_matches_required_;

  // State
  nm::OccupancyGrid::ConstPtr grid_;
  sm::LaserScan::ConstPtr scan_;
  vector<RefScan> ref_scans_;
  boost::optional<gm::Pose> last_pose_;

  // Flirtlib objects
  boost::shared_ptr<SimpleMinMaxPeakFinder> peak_finder_;
  boost::shared_ptr<HistogramDistance<double> > histogram_dist_;
  boost::shared_ptr<Detector> detector_;
  boost::shared_ptr<DescriptorGenerator> descriptor_;
  boost::shared_ptr<RansacFeatureSetMatcher> ransac_;
  
  // Ros objects
  tf::TransformListener tf_;
  ros::Subscriber scan_sub_;
  ros::Publisher marker_pub_;
  ros::Timer exec_timer_;

};


/************************************************************
 * Initialization
 ***********************************************************/

template <class T>
T getPrivateParam (const string& name)
{
  ros::NodeHandle nh("~");
  T val;
  const bool found = nh.getParam(name, val);
  ROS_ASSERT_MSG (found, "Could not find parameter %s", name.c_str());
  ROS_DEBUG_STREAM_NAMED ("init", "Initialized " << name << " to " << val);
  return val;
}

template <class T>
T getPrivateParam (const string& name, const T& default_val)
{
  ros::NodeHandle nh("~");
  T val;
  nh.param(name, val, default_val);
  ROS_DEBUG_STREAM_NAMED ("init", "Initialized " << name << " to " << val <<
                          "(default was " << default_val << ")");
  return val;
}

sm::LaserScan getScannerParams ()
{
  sm::LaserScan params;
  const double PI = 3.14159265;
  params.angle_min = -PI/2;
  params.angle_max = PI/2;
  params.angle_increment = PI/180;
  params.range_max = 10;

  return params;
}

SimpleMinMaxPeakFinder* createPeakFinder ()
{
  return new SimpleMinMaxPeakFinder(0.34, 0.001);
}

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

Node::Node () :
  map_file_(getPrivateParam<string>("map_file")),
  resolution_(getPrivateParam<double>("resolution")),
  scanner_params_(getScannerParams()),
  ref_pos_resolution_(getPrivateParam<double>("ref_pos_resolution")),
  ref_angle_resolution_(getPrivateParam<double>("ref_angle_resolution")),
  x0_(getPrivateParam<double>("x0", -1e9)),
  x1_(getPrivateParam<double>("x1", 1e9)),
  y0_(getPrivateParam<double>("y0", -1e9)),
  y1_(getPrivateParam<double>("y1", 1e9)),
  num_matches_required_(getPrivateParam<int>("num_matches_required", 10)),
  grid_(gu::loadGrid(map_file_, resolution_)),

  peak_finder_(createPeakFinder()),
  histogram_dist_(new EuclideanDistance<double>()),
  detector_(createDetector(peak_finder_.get())),
  descriptor_(createDescriptor(histogram_dist_.get())),
  ransac_(new RansacFeatureSetMatcher(0.0599, 0.95, 0.4, 0.4,
                                           0.0384, false)),

  scan_sub_(nh_.subscribe("scan0", 10, &Node::scanCB, this)),
  marker_pub_(nh_.advertise<vm::Marker>("visualization_marker", 10)),
  exec_timer_(nh_.createTimer(ros::Duration(0.2), &Node::mainLoop, this))
{
  initializeRefScans();
}


/************************************************************
 * Creation of reference scans
 ***********************************************************/

void Node::initializeRefScans ()
{
  ROS_INFO ("Initializing reference scans");
  const unsigned skip = ref_pos_resolution_/grid_->info.resolution;
  for (unsigned x=0; x<grid_->info.width; x+=skip)
  {
    for (unsigned y=0; y<grid_->info.height; y+=skip)
    {
      for (double theta=0; theta<6.28; theta+=ref_angle_resolution_)
      {
        // Get position and see if it's within limits
        gm::Pose pose;
        const gu::Cell c(x,y);
        pose.position = gu::cellCenter(grid_->info, c);
        pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        if ((pose.position.x > x1_) || (pose.position.x < x0_) ||
            (pose.position.y > y1_) || (pose.position.y < y0_))
          continue;
        if (grid_->data[cellIndex(grid_->info, c)]!=gu::UNOCCUPIED)
          continue;
        ROS_INFO_THROTTLE (0.1, "Pos: %.2f, %.2f", pose.position.x,
                           pose.position.y);
        
        // Simulate scan
        sm::LaserScan::Ptr scan = gu::simulateRangeScan(*grid_, pose, scanner_params_, true);

        // Convert to flirtlib and extract features
        boost::shared_ptr<LaserReading> reading = fromRos(*scan);
        InterestPointVec pts;
        detector_->detect(*reading, pts);
        BOOST_FOREACH (InterestPoint* p, pts)
          p->setDescriptor(descriptor_->describe(*p, *reading));
        
        // Save
        RefScan ref(scan, pose, pts);
        ref_scans_.push_back(ref);
      }
    }
  }
  ROS_INFO_STREAM ("Generated " << ref_scans_.size() << " reference scans");
}


/************************************************************
 * Visualization
 ***********************************************************/


/************************************************************
 * Callbacks
 ***********************************************************/


void Node::scanCB (sm::LaserScan::ConstPtr scan)
{
  Lock l(mutex_);
  scan_ = scan;
}


/************************************************************
 * Main loop
 ***********************************************************/


gm::Pose Node::getPose ()
{
  tf::StampedTransform trans;
  tf_.lookupTransform("/map", "pose0", ros::Time(), trans);
  gm::Pose pose;
  tf::poseTFToMsg(trans, pose);
  return pose;
}

bool closeTo (const gm::Pose& p1, const gm::Pose& p2)
{
  const double TOL=1e-3;
  const double dx = p1.position.x - p2.position.x;
  const double dy = p1.position.y - p2.position.y;
  const double dt = tf::getYaw(p1.orientation) - tf::getYaw(p2.orientation);
  return fabs(dx)<TOL && fabs(dy)<TOL && fabs(dt)<TOL;
}

void Node::mainLoop (const ros::TimerEvent& e)
{
  try
  {
    // Getting pose is the part that can throw exceptions
    const gm::Pose current_pose = getPose();
    const double theta = tf::getYaw(current_pose.orientation);
    const double x=current_pose.position.x;
    const double y=current_pose.position.y;

    Lock l(mutex_);
    if (last_pose_ && closeTo(*last_pose_, current_pose))
      return;
    if (!scan_)
      return;

    ROS_INFO("Matching scan at %.2f, %.2f, %.2f", x, y, theta);
    // Extract features for this scan
    InterestPointVec pts;
    boost::shared_ptr<LaserReading> reading = fromRos(*scan_);
    detector_->detect(*reading, pts);
    BOOST_FOREACH (InterestPoint* p, pts) 
      p->setDescriptor(descriptor_->describe(*p, *reading));
    marker_pub_.publish(interestPointMarkers(pts, current_pose, 0));

    // Match
    vector<gm::Pose> match_poses;
    BOOST_FOREACH (const RefScan& ref_scan, ref_scans_) 
    {
      Correspondences matches;
      OrientedPoint2D transform;
      ransac_->matchSets(ref_scan.pts, pts, transform, matches);
      if (matches.size() > num_matches_required_)
      {
        ROS_INFO ("Found %zu matches with ref scan at %.2f, %.2f, %.2f.  "
                  "Current pose is %.2f, %.2f, %.2f.", matches.size(),
                  ref_scan.pose.position.x, ref_scan.pose.position.y,
                  tf::getYaw(ref_scan.pose.orientation), x, y, theta);
        match_poses.push_back(ref_scan.pose);
      }
    }
    ROS_INFO ("Done matching");

    BOOST_FOREACH (const vm::Marker& m, poseMarkers(match_poses))
      marker_pub_.publish(m);
    last_pose_ = current_pose;
  }

  catch (tf::TransformException& e)
  {
    ROS_INFO ("Skipping because of tf exception");
  }
}




} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "flirtlib_ros_test");
  flirtlib_ros::Node node;
  ros::spin();
  return 0;
}
