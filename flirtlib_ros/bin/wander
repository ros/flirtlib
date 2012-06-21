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

# Wander around in the hope of finding a matching scan

import roslib
roslib.load_manifest('flirtlib_ros')

import sys
import tf.transformations as tr
import rospy
import tf
import random
import move_base_msgs.msg as mbm
import actionlib as al
from rospy import loginfo
import geometry_msgs.msg as gm
from math import sin, cos
import flirtlib_ros.msg as msg
import std_srvs.srv

class Node(object):

    def __init__(self):
        self.is_localized = False
        self.state_sub = rospy.Subscriber('startup_loc_state',
                                          msg.ExecutiveState,
                                          self.exec_state_cb)
        self.tf = tf.TransformListener()
        self.nav_client = al.SimpleActionClient('move_base_local/move_base',
                                                mbm.MoveBaseAction)
        loginfo("Waiting for move base server")
        self.nav_client.wait_for_server()
        loginfo("Waiting for flirtlib reset service")
        SRV = 'flirtlib_place_rec/reset_state'
        rospy.wait_for_service(SRV)
        reset = rospy.ServiceProxy(SRV, std_srvs.srv.Empty)
        reset()
        loginfo("Wander node initialized")

    def odom_pose(self):
        trans, rot = self.tf.lookupTransform('odom_combined',
                                             'base_footprint',
                                             rospy.Time())
        yaw = tr.euler_from_quaternion(rot)[2]
        return (trans[0], trans[1], yaw)

    def run(self):
        while not rospy.is_shutdown():
            if self.is_localized:
                break
            self.move_new_pose()
            

    def exec_state_cb(self, m):
        if not self.is_localized and m.localization_badness<0.25:
            self.is_localized = True
        

    def random_rotation(self):
        x, y, theta = self.odom_pose()
        M = 1
        dtheta = random.uniform(-M, M)
        if abs(dtheta)<0.25*M:
            if dtheta<0:
                dtheta = -1-dtheta
            else:
                dtheta = 1-dtheta
        loginfo("Rotating {0}".format(dtheta))
        return self.move_to(x, y, theta+dtheta)

    def forward(self):
        "Move forward a random distance"
        M = 1
        dist = random.uniform(0, 1)
        loginfo("Moving forward {0}".format(dist))
        x, y, theta = self.odom_pose()
        dx = dist*cos(theta)
        dy = dist*sin(theta)
        return self.move_to(x+dx, y+dy, theta)
    

    def move_to(self, x, y, theta):
        g = mbm.MoveBaseGoal()
        g.target_pose.header.frame_id = 'odom_combined'
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position = gm.Point(x, y, 0)
        q = tr.quaternion_from_euler(0, 0, theta)
        g.target_pose.pose.orientation = gm.Quaternion(*q)
        loginfo("Nav goal is {0}".format(g.target_pose.pose))
        res = self.nav_client.send_goal_and_wait(\
            g, execute_timeout=rospy.Duration(2.0))
        loginfo("Nav result was {0}".format(res))
        return res

    def move_new_pose(self):
        if random.random()>0.5:
            return self.forward()
        else:
            return self.random_rotation()
   

if __name__=='__main__':
    rospy.init_node('kidnapped_robot_wander')
    n = Node()
    n.run()
    
