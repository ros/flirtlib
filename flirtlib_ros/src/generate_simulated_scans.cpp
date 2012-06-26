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
#include <boost/program_options.hpp>

namespace sm=sensor_msgs;
namespace gm=geometry_msgs;
namespace fr=flirtlib_ros;
namespace mr=mongo_ros;
namespace nm=nav_msgs;
namespace gu=occupancy_grid_utils;
namespace po=boost::program_options;

using std::cerr;
using std::endl;
using std::string;

const double PI = 3.14159265;

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

  // Parse args
  double resolution;
  gm::Pose origin;
  double origin_theta;
  double disc;
  string db_name;
  int num_angles;
  string filename;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "Print help message")
    ("resolution,r", po::value<double>(&resolution),
     "Map resolution (required)")
    ("origin_x,x", po::value<double>(&origin.position.x)->default_value(0),
     "Origin x")
    ("origin_y,y", po::value<double>(&origin.position.y)->default_value(0),
     "Origin y")
    ("origin_theta,t", po::value<double>(&origin_theta)->default_value(0),
     "Origin yaw")
    ("discretization,z", po::value<double>(&disc)->default_value(1.0),
     "Scan discretization (default 1.0)")
    ("database_name,d", po::value<string>(&db_name),
     "Name of database (required)")
    ("filename,f", po::value<string>(&filename), "Map file (required)")
    ("num_angles,a", po::value<int>(&num_angles)->default_value(4),
     "Number of angles considered at each position")
    ;
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("resolution") || !vm.count("filename") ||
      !vm.count("database_name"))
  {
    cerr << desc << endl;
    return 1;
  }
  
  // Load occupancy grid from file
  origin.orientation = tf::createQuaternionMsgFromYaw(origin_theta);
  nm::OccupancyGrid::Ptr grid = gu::loadGrid(filename, resolution, origin);
  cerr << "Loaded a " << grid->info.height << "x" << grid->info.width <<
    " map at " << grid->info.resolution << " m/cell with origin " <<
    origin;
  gm::Polygon grid_poly = gu::gridPolygon(grid->info);
  double x_min=1e9, y_min=1e9, x_max=-1e9, y_max=-1e9;
  ROS_ASSERT (grid_poly.points.size()==4);
  BOOST_FOREACH (const gm::Point32& corner, grid_poly.points) 
  {
    if (corner.x < x_min)
      x_min = corner.x;
    if (corner.x > x_max)
      x_max = corner.x;
    if (corner.y < y_min)
      y_min = corner.y;
    if (corner.y > y_max)
      y_max = corner.y;
  }
  ROS_ASSERT (x_min<1e9 && y_min<1e9 && x_max>-1e9 && y_max>-1e9);
  ROS_ASSERT_MSG (origin_theta==0, "Can't handle rotated maps");
  
  // Set up scan info
  // \todo PR2 specific values: allow changing
  sm::LaserScan scan_info;
  scan_info.angle_min = -2.26892805099;
  scan_info.angle_max = 2.26456475258;
  scan_info.angle_increment = 0.00436332309619;
  scan_info.range_max = 60.0;
  
  // Set up db connection and remove any existing scans
  mr::dropDatabase(db_name, "localhost", 27017);
  mr::MessageCollection<fr::RefScanRos> scans(db_name, "scans", "localhost");

  // Set up feature extractor
  fr::FlirtlibFeatures features;

  // Note loop is using the above assumption that origin_theta = 0
  for (double x=x_min+disc; x<x_max; x+=disc)
  {
    cerr << "x = " << x << endl;
    for (double y=y_min+disc; y<y_max; y+=disc)
    {
      gm::Point pt;
      pt.x = x;
      pt.y = y;
      if (grid->data[gu::pointIndex(grid->info, pt)]!=gu::UNOCCUPIED)
        continue;
      cerr << "  y = " << y << endl;
      for (double theta=0; theta<2*PI; theta += 2*PI/num_angles)
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
