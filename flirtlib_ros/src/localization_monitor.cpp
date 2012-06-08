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
 * Implementation of localization_monitor.h
 *
 * \author Bhaskara Marthi
 */

#include <flirtlib_ros/localization_monitor.h>

namespace flirtlib_ros
{

namespace sm=sensor_msgs;
namespace gm=geometry_msgs;
namespace gu=occupancy_grid_utils;
namespace nm=nav_msgs;

ScanPoseEvaluator::ScanPoseEvaluator (const nm::OccupancyGrid& g) :
  geom_(g.info), distances_(gu::distanceField(g, 0.4))
{
  ROS_INFO ("Scan pose evaluator initialized");
}


float ScanPoseEvaluator::operator() (const sm::LaserScan& scan,
                                     const gm::Pose& pose) const
{
  // Scanner pose
  const double theta0 = tf::getYaw(pose.orientation);
  const double x0 = pose.position.x;
  const double y0 = pose.position.y;

  std::vector<double> distances;

  // Iterate over range readings, and get the coordinates of the 
  // corresponding point in the map frame for the given sensor pose
  for (size_t i=0; i<scan.ranges.size(); i++)
  {
    const double theta = theta0 + scan.angle_min + i*scan.angle_increment;
    const double r = scan.ranges[i];
    gm::Point p;
    p.x = x0 + cos(theta)*r;
    p.y = y0 + sin(theta)*r;
    p.z = 0;
    
    // Now figure out the distance to the closest obstacle in the map
    // If the point's off the map, use a large value (we're taking a median, 
    // so it doesn't matter)
    const gu::Cell c = gu::pointCell(geom_, p);
    const double d = withinBounds(geom_, c) ? distances_[c] : 1e9;
    distances.push_back(d);
  }
  
  // Return the median distance
  sort(distances.begin(), distances.end());
  const size_t mid = distances.size()/2;
  return distances[mid];
}

} // namespace
