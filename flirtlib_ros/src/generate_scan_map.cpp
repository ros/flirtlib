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
 * Node that listens to scans and localization, and generates a scan map,
 * i.e., a set of scans that cover the x-y-theta space pretty well, with
 * extracted features
 *
 * \author Bhaskara Marthi
 */

#include <flirtlib_ros/conversions.h>
#include <ros/ros.h>

namespace flirtlib_ros
{

class Node
{
public:
  
  Node();
  void scanCB (sm::LaserScan::ConstPtr scan);
  
private:

  // Needed during init
  boost::mutex mutex_;
  ros::NodeHandle nh_;

  // Parameters
  const double x_inc_, y_inc_, theta_inc_;

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
  mr::MessageCollection<RefScanRos> scans_;
  
};

/************************************************************
 * Initialization
 ***********************************************************/

Node::Node () :
{
  ROS_ASSERT_MSG(scans_.count()==0, "Scan collection was not empty");
  // Create index by x, y
}

/************************************************************
 * Processing scans
 ***********************************************************/

void Node::scanCB (sm::LaserScan::ConstPtr scan)
{
  const gm::Pose pose = getCurrentPose();
  if (haveNearbyPose(scans_, pose))
    return;

  // Now guaranteed no nearby pose
  scans_.insert(extractFeatures(scan, pose),
                mr::Metadata("x", pose.position.x, "y", pose.position.y));
  
}

} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "generate_scan_map");
  Node node;
  ros::spin();
  return 0;
}
