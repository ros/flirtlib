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
 * Load occupancy grid from file, simulate fake scan(s), visualize
 * results of flirtlib
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

namespace sm=sensor_msgs;
namespace gu=occupancy_grid_utils;
namespace vm=visualization_msgs;
namespace gm=geometry_msgs;
namespace nm=nav_msgs;

using std::string;
using std::vector;

typedef boost::mutex::scoped_lock Lock;
typedef vector<InterestPoint*> InterestPointVec;

namespace flirtlib_ros
{



class Node
{
public:
  Node ();

  void mainLoop (const ros::TimerEvent& e);

private:

  ros::NodeHandle nh_;
  boost::mutex mutex_;
  
  const string map_file_;
  const double resolution_;
  nm::OccupancyGrid::Ptr grid_;
  const sm::LaserScan scanner_params_;

  boost::shared_ptr<SimpleMinMaxPeakFinder> peak_finder_;
  boost::shared_ptr<Detector> detector_;
  ros::Publisher scan_pubs_[2];
  ros::Publisher grid_pub_;
  ros::Publisher marker_pub_;
  tf::TransformListener tf_;
  ros::Timer exec_timer_;

};

template <class T>
T getPrivateParam (const string& name, const T& default_val)
{
  ros::NodeHandle nh("~");
  T val;
  nh.param(name, val, default_val);
  ROS_DEBUG_STREAM_NAMED ("init", "Setting parameter " << name << " to " <<
                          val << " (default was " << default_val << ")");
  return val;
}

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

Node::Node () :
  map_file_(getPrivateParam<string>("map_file")),
  resolution_(getPrivateParam<double>("resolution")),
  grid_(gu::loadGrid(map_file_, resolution_)),
  scanner_params_(getScannerParams()),
  peak_finder_(createPeakFinder()),
  detector_(createDetector(peak_finder_.get())),
  grid_pub_(nh_.advertise<nm::OccupancyGrid>("grid", 10, true)),
  marker_pub_(nh_.advertise<vm::Marker>("visualization_marker", 10)),
  exec_timer_(nh_.createTimer(ros::Duration(0.2), &Node::mainLoop, this))
{
  for (unsigned i=0; i<2; i++)
  {
    const string topic = "scan" + boost::lexical_cast<string>(i);
    scan_pubs_[i] = nh_.advertise<sm::LaserScan>(topic, 10, true);
  }
  grid_->header.frame_id = "/map";
  grid_->header.stamp = ros::Time::now();
  ROS_INFO ("Loaded a %ux%u grid at %.4f m/cell",
            grid_->info.height, grid_->info.width,
            grid_->info.resolution);
  grid_pub_.publish(grid_);
}

typedef vector<std_msgs::ColorRGBA> ColorVec;
ColorVec initColors ()
{
  ColorVec colors(2);
  colors[0].r = 0.5;
  colors[0].g = 1.0;
  colors[0].a = 1.0;
  colors[1].r = 1.0;
  colors[1].g = 1.0;
  colors[1].a = 1.0;
  return colors;
}

// Generate visualization markers for the interest points
// id is 0 or 1, and controls color and orientation to distinguish between
// the two scans
vm::Marker interestPointMarkers (const InterestPointVec& pts, const unsigned id)
{
  static ColorVec colors = initColors();
  vm::Marker m;
  m.header.frame_id = "/map";
  m.header.stamp = ros::Time::now();
  m.ns = "flirtlib";
  m.id = id;
  m.type = vm::Marker::LINE_LIST;
  m.scale.x = 0.05;
  m.color = colors[id];
  BOOST_FOREACH (const InterestPoint* p, pts) 
  {
    const double x0 = p->getPosition().x;
    const double y0 = p->getPosition().y;
    const double d = 0.1;
    double dx[4];
    double dy[4];
    if (id==0)
    {
      dx[0] = dx[3] = -d;
      dx[1] = dx[2] = d;
      dy[0] = dy[1] = -d;
      dy[2] = dy[3] = d;
    }
    else
    {
      ROS_ASSERT(id==1);
      const double r2 = sqrt(2);
      dx[0] = dx[2] = dy[1] = dy[3] = 0;
      dx[1] = dy[0] = -r2*d;
      dx[3] = dy[2] = r2*d;
    }

    for (unsigned i=0; i<4; i++)
    {
      unsigned j = (i==0) ? 3 : i-1;
      gm::Point pt, pt2;
      pt.x = x0+dx[i];
      pt.y = y0+dy[i];
      pt2.x = x0+dx[j];
      pt2.y = y0+dy[j];
      m.points.push_back(pt);
      m.points.push_back(pt2);      
    }
  }

  return m;
}

void Node::mainLoop (const ros::TimerEvent& e)
{
  try
  {
    gm::Pose pose[2];
    sm::LaserScan::Ptr scan[2];
    InterestPointVec pts[2];

    for (unsigned i=0; i<2; i++)
    {
      // Get the poses
      // This part can throw a tf exception
      tf::StampedTransform trans;
      const string frame = "pose"+boost::lexical_cast<string>(i);
      tf_.lookupTransform("/map", frame, ros::Time(), trans);
      tf::poseTFToMsg(trans, pose[i]);

      // Simulate scans and convert to flirtlib
      scan[i] = gu::simulateRangeScan(*grid_, pose[i],
                                      scanner_params_, true);
      scan[i]->header.frame_id = frame;
      scan[i]->header.stamp = ros::Time::now();
      scan_pubs_[i].publish(scan[i]);
      boost::shared_ptr<LaserReading> reading = fromRos(*scan[i], pose[i]);

      // Interest point detection
      detector_->detect(*reading, pts[i]);
      marker_pub_.publish(interestPointMarkers(pts[i], i));
    }


    // Free memory
    for (unsigned i=0; i<2; i++)
    {
      BOOST_FOREACH (const InterestPoint* p, pts[i]) 
        delete p;
    }
  }
  catch (tf::TransformException& e)
  {
    ROS_INFO("Skipping frame due to transform exception");
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
