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
 * Implementation for conversions.h
 *
 * \author Bhaskara Marthi
 */

#include <flirtlib_ros/conversions.h>
#include <tf/transform_datatypes.h>
#include <boost/foreach.hpp>

namespace sm=sensor_msgs;
namespace gm=geometry_msgs;
namespace vm=visualization_msgs;

using std::vector;

typedef boost::shared_ptr<LaserReading> LaserPtr;
typedef vector<double> DoubleVec;
typedef vector<InterestPoint*> InterestPointVec;

namespace flirtlib_ros
{

/// Convert a ROS laser scan message to a flirtlib scan
LaserPtr fromRos(const sm::LaserScan& m)
{
  const unsigned n = m.ranges.size();
  DoubleVec angles(n);
  DoubleVec ranges(n);
  for (unsigned i=0; i<n; i++)
  {
    angles[i] = m.angle_min + i*m.angle_increment;
    ranges[i] = m.ranges[i];
  }
  LaserPtr reading(new LaserReading(angles, ranges, m.header.stamp.toSec(),
                                    "ros_laser", "ros_robot"));
  return reading;
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



vector<vm::Marker> poseMarkers (const vector<gm::Pose>& poses)
{
  vector<vm::Marker> markers;

  for (unsigned id=0; id<100; id++)
  {
    vm::Marker m;
    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();
    m.ns="flirtlib.poses";
    m.id = id;
    m.type = vm::Marker::ARROW;
    m.color.r = m.color.b = .15;
    m.color.g = .65;
    m.color.a = 1;
    m.scale.x = m.scale.y = 2.0;
    m.scale.z = 1.0;
    if (id < poses.size())
    {
      m.pose = poses[id];
    }
    else
    {
      m.pose.orientation.w = 1.0;
      m.action = vm::Marker::DELETE;
    }

    markers.push_back(m);
  }

  return markers;
}

gm::Point toPoint (const btVector3 p)
{
  gm::Point pt;
  pt.x = p.x();
  pt.y = p.y();
  pt.z = p.z();
  return pt;
}


// Generate visualization markers for the interest points
// id is 0 or 1, and controls color and orientation to distinguish between
// the two scans
vm::Marker interestPointMarkers (const InterestPointVec& pts, const gm::Pose& pose, const unsigned id)
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





} // namespace
