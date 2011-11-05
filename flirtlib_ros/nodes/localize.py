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

# Script frontend for flirtlib startup localization

import roslib; roslib.load_manifest('flirtlib_ros')
import rospy
import geometry_msgs.msg as gm
import std_msgs.msg as std
import threading
import sys
import actionlib as al
from flirtlib_ros import msg



class Node:

    def __init__(self):
        self.lock = threading.Lock()
        self.pose = None
        self.done = False
        self.client = al.SimpleActionClient('rotate_in_place',
                msg.RotateInPlaceAction)
        rospy.loginfo("Waiting for action client")
        self.client.wait_for_server()
        rospy.loginfo("Found")
        self.rotating = False
        
        with self.lock:
            self.pose_sub = rospy.Subscriber('pose_estimate', gm.PoseStamped,
                                            self.save_pose)
            self.pub = rospy.Publisher('initialpose', gm.PoseWithCovarianceStamped, latch=True)

    def save_pose(self, m):
        with self.lock:
            if not self.rotating and not self.done:
                self.pub.publish(header=m.header, pose=gm.PoseWithCovariance(pose=m.pose))
                self.done = True
                rospy.sleep(1.0)
                rospy.loginfo("Found a good pose; exiting")
                rospy.signal_shutdown("Pose found")
                

    def metaphorically_spin(self):
        for i in xrange(15):
            rospy.sleep(1.0)
            if rospy.is_shutdown():
                break
        if not rospy.is_shutdown():
            rospy.loginfo("Didn't find a good localization within 10 seconds")
        

    def literally_spin(self):
        for i in xrange(10):
            rospy.sleep(1.5)
            if rospy.is_shutdown():
                break
            yaw = i*0.628
            rospy.loginfo ("Rotating to {0}".format(yaw))
            goal = msg.RotateInPlaceGoal(target=yaw, tol=0.1)
            goal.header.frame_id='odom_combined'
            self.rotating = True
            self.client.send_goal_and_wait(goal)
            rospy.sleep(0.1)
            self.rotating = False
        if not rospy.is_shutdown():
            rospy.loginfo("Didn't find a good localization after rotating")
        self.client.cancel_all_goals()


    
    

    
if __name__ == "__main__":
    rospy.init_node('global_localization')
    node = Node()
    if '--active' in sys.argv:
        node.literally_spin()
    else:
        node.metaphorically_spin()
