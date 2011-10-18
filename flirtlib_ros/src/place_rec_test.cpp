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
typedef std::pair<InterestPoint*, InterestPoint*> Correspondence;
typedef vector<Correspondence> Correspondences;

namespace flirtlib_ros
{

/************************************************************
 * Utilities
 ***********************************************************/

gm::Point toPoint (const btVector3 p)
{
  gm::Point pt;
  pt.x = p.x();
  pt.y = p.y();
  pt.z = p.z();
  return pt;
}

struct RefScan
{
  sm::LaserScan::ConstPtr scan;
  gm::Pose pose;

  RefScan (sm::LaserScan::ConstPtr scan, const gm::Pose& pose) :
    scan(scan), pose(pose)
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

  // Needed during initialization
  boost::mutex mutex_;
  ros::NodeHandle nh_;

  // Parameters 
  const string map_file_;
  const double resolution_;
  const sm::LaserScan scanner_params_;

  // State
  sm::LaserScan::ConstPtr scan_;
  vector<RefScan> ref_scans_;

  // Flirtlib objects
  boost::shared_ptr<SimpleMinMaxPeakFinder> peak_finder_;
  boost::shared_ptr<HistogramDistance<double> > histogram_dist_;
  boost::shared_ptr<Detector> detector_;
  boost::shared_ptr<DescriptorGenerator> descriptor_;
  boost::shared_ptr<RansacFeatureSetMatcher> ransac_;
  
  // Ros objects
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

/************************************************************
 * Creation of reference scans
 ***********************************************************/

void Node::initializeRefScans ()
{
  ROS_INFO ("Initializing reference scans");
}


/************************************************************
 * Visualization
 ***********************************************************/


// Generate visualization markers for the interest points
// id is 0 or 1, and controls color and orientation to distinguish between
// the two scans
vm::Marker interestPointMarkers (const InterestPointVec& pts, const unsigned id,
                                 const gm::Pose& pose)
{
  tf::Transform trans;
  tf::poseMsgToTF(pose, trans);
  static ColorVec colors = initColors();
  vm::Marker m;
  m.header.frame_id = "/map";
  m.header.stamp = ros::Time::now();
  m.ns = "flirtlib";
  m.id = id;
  m.type = vm::Marker::LINE_LIST;
  m.scale.x = 0.03;
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
      const unsigned j = (i==0) ? 3 : i-1;
      const btVector3 pt0(x0+dx[i], y0+dy[i], 0);
      const btVector3 pt1(x0+dx[j], y0+dy[j], 0);
      m.points.push_back(toPoint(trans*pt0));
      m.points.push_back(toPoint(trans*pt1));
    }
  }

  return m;
}

// Generate markers to visualize correspondences between two scans
vm::Marker correspondenceMarkers (const Correspondences& correspondences,
                                  const gm::Pose& p0, const gm::Pose& p1)
{
  vm::Marker m;
  m.header.frame_id = "/map";
  m.header.stamp = ros::Time::now();
  m.ns = "flirtlib";
  m.id = 42;
  m.type = vm::Marker::LINE_LIST;
  m.scale.x = 0.05;
  m.color.r = m.color.a = 1.0;
  m.color.g = 0.65;
  tf::Pose pose0, pose1;
  tf::poseMsgToTF(p0, pose0);
  tf::poseMsgToTF(p1, pose1);

  BOOST_FOREACH (const Correspondence& c, correspondences) 
  {
    const btVector3 pt0(c.first->getPosition().x, c.first->getPosition().y, 0.0);
    const btVector3 pt1(c.second->getPosition().x, c.second->getPosition().y, 0.0);
    m.points.push_back(toPoint(pose0*pt0));
    m.points.push_back(toPoint(pose1*pt1));
  }

  return m;
}

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


void Node::mainLoop (const ros::TimerEvent& e)
{
  Lock l(mutex_);
  ROS_INFO ("Main loop");
}



} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "flirtlib_ros_test");
  flirtlib_ros::Node node;
  ros::spin();
  return 0;
}
