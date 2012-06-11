#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Bhaskara Marthi

# Script to visualize saved scans


import roslib
roslib.load_manifest('flirtlib_ros')

import sys
import geometry_msgs.msg as gm
import sensor_msgs.msg as sm
import rospy
import tf.transformations as trans
from math import sin, cos
import mongo_ros as mr
import flirtlib_ros.msg as fm

def scan2cloud(scan, pose):
    x0 = pose.position.x
    y0 = pose.position.y
    print "Angle is ", pose.orientation
    theta0 = trans.euler_from_quaternion([pose.orientation.x, pose.orientation.y,
                                        pose.orientation.z, pose.orientation.w])[2]
    print "Base pos is {0}, {1}, {2}".format(x0, y0, theta0)
    cloud = sm.PointCloud()
    cloud.header.frame_id='/map'
    cloud.header.stamp = rospy.Time.now()
    cloud.points = []
    for i, r in enumerate(scan.ranges):
        theta = theta0 + scan.angle_min + i*scan.angle_increment 
        p = gm.Point32(x0+cos(theta)*r, y0+sin(theta)*r, 0)
        cloud.points.append(p)

    return cloud


def main():
    rospy.init_node('test_scan')
    c=mr.MessageCollection('flirtlib_place_rec', 'scans', fm.RefScanRos,
                           db_host='mongo.willowgarage.com')
    l = list(c.query({}))
    if len(sys.argv!=2):
        print "Usage: {0} SCAN_NUM".format(sys.argv[0])
    i = int(sys.argv[1])
    print "Visualizing scan ", i
    msg = l[i][0]
    cl = scan2cloud(msg.scan, msg.pose)
    pub = rospy.Publisher('saved_cloud', sm.PointCloud, latch=True)
    pub.publish(cl)
    rospy.spin()

if __name__=='__main__':
    sys.exit(main())



