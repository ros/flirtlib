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
 * Utility that generates scans from a pgm file representing a map
 *
 * \author Bhaskara Marthi
 */

#include <occupancy_grid_utils/file.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <tf/transform_datatypes.h>
#include <mongo_ros/message_collection.h>
#include <mongo_ros/mongo_ros.h>
#include <flirtlib_ros/flirtlib.h>
#include <flirtlib_ros/conversions.h>

namespace sm=sensor_msgs;
namespace gm=geometry_msgs;
namespace fr=flirtlib_ros;
namespace mr=mongo_ros;
namespace nm=nav_msgs;
namespace gu=occupancy_grid_utils;

using std::cerr;
using std::endl;

const double PI = 3.14159265;
const unsigned NUM_ANGLES = 8;

gm::Pose makePose (const double x, const double y, const double theta)
{
  gm::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation = tf::createQuaternionMsgFromYaw(theta);
  return p;
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "generate_simulated_scans");

  // Parse args: todo remove ros stuff and use program_options
  if (argc!=5)
  {
    cerr << "Usage: " << argv[0] << " MAPFILE RESOLUTION DISCRETIZATION DB\n";
    return 1;
  }
  const double resolution = atof(argv[2]);
  const double discretization = atof(argv[3]);

  // Load occupancy grid from file
  nm::OccupancyGrid::Ptr grid = gu::loadGrid(argv[1], resolution);
  cerr << "Loaded a " << grid->info.height << "x" << grid->info.width <<
    " map at " << grid->info.resolution << " m/cell\n";
  const double x_max = grid->info.width*resolution;
  const double y_max = grid->info.height*resolution;
  
  // Set up scan info
  // \todo PR2 specific values: allow changing
  sm::LaserScan scan_info;
  scan_info.angle_min = -2.26892805099;
  scan_info.angle_max = 2.26456475258;
  scan_info.angle_increment = 0.00436332309619;
  scan_info.range_max = 60.0;
  
  // Set up db connection
  mr::dropDatabase(argv[4], "localhost", 27017);
  mr::MessageCollection<fr::RefScanRos> scans(argv[4], "scans", "localhost");

  // Set up feature extractor
  fr::FlirtlibFeatures features;

  for (double x=0; x<x_max; x+=discretization)
  {
    cerr << "x = " << x << endl;
    for (double y=0; y<y_max; y+=discretization)
    {
      gm::Point pt;
      pt.x = x;
      pt.y = y;
      cerr << "  y = " << y << endl;
      if (grid->data[gu::pointIndex(grid->info, pt)]!=gu::UNOCCUPIED)
      {
        cerr << "    skipping\n";
        continue;
      }
      for (double theta=0; theta<2*PI; theta += 2*PI/NUM_ANGLES)
      {
        const gm::Pose& p = makePose(x, y, theta);
        sm::LaserScan::Ptr scan = gu::simulateRangeScan(*grid, p, scan_info);
        fr::InterestPointVec pts = features.extractFeatures(scan);
        sm::LaserScan::Ptr stripped_scan(new sm::LaserScan(*scan));
        stripped_scan->ranges.clear();
        stripped_scan->intensities.clear();
        fr::RefScan ref_scan(stripped_scan, p, pts);
        scans.insert(toRos(ref_scan),
                      mr::Metadata("x", x, "y", y, "theta", theta));
      }
    }
  }
}
