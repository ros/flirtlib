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
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace flirtlib_ros
{

namespace sm=sensor_msgs;
namespace gm=geometry_msgs;
namespace nm=nav_msgs;

typedef boost::mutex::scoped_lock Lock;

class Node
{
public:

  Node ();

  void scanCB (const sm::LaserScan& scan);
  
  void mapCB (const nm::OccupancyGrid& g);

private:

  boost::mutex mutex_;
  ros::NodeHandle nh_;

  boost::shared_ptr<ScanPoseEvaluator> evaluator_;

  tf::TransformListener tf_;
  ros::Subscriber scan_sub_;
  ros::Subscriber map_sub_;
};


// Constructor sets up ros comms
Node::Node () :
  scan_sub_(nh_.subscribe("base_scan", 10, &Node::scanCB, this)),
  map_sub_(nh_.subscribe("map", 10, &Node::mapCB, this))
{}


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

// Given current scan and pose, evaluate how good our localization is
void Node::scanCB (const sm::LaserScan& scan)
{
  // First, look up the sensor pose at the time of the scan
  tf_.waitForTransform("/map", scan.header.frame_id, scan.header.stamp,
                       ros::Duration(0.5));
  tf::StampedTransform trans;
  try
  {
    tf_.lookupTransform("/map", scan.header.frame_id, scan.header.stamp,
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
  const double dist = (*evaluator_)(scan, toMsg(trans));
  ROS_INFO_THROTTLE (1.0, "Median distance is %.4f", dist);
                           
}

} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "localization_monitor_node");
  flirtlib_ros::Node node;
  ros::spin();
  return 0;
}
