#!/usr/bin/env python
"""--------------------------------------------------------------------
Copyright (c) 2018, Kinova Robotics inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\Author Longfei Zhao
\brief  Demo prepared for Movo2
\Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import threading
import numpy

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from movo.system_defines import TRACTOR_REQUEST
from movo_msgs.msg import ConfigCmd
from movo_msgs.msg import FaceFound

from movo_action_clients.move_base_action_client import MoveBaseActionClient
from geometry_msgs.msg import Pose2D


class MoveCloser:
    def __init__(self):

        # "translation" or "rotation"
        # self._base_motion_mode = ''
        #
        # self._Tsampling = 0.01
        # self._sampling_time = rospy.get_rostime()
        # self._base_x_speed_filter = LowPassFilter(k=1.0, Tconst=0.2, Tsampling=0.01)
        # self._base_y_speed_filter = LowPassFilter(k=1.0, Tconst=0.2, Tsampling=0.01)
        # self._base_theta_z_speed_filter = LowPassFilter(k=1.0, Tconst=0.1, Tsampling=0.01)
        #
        # self._is_first_run = True
        # self._start_time = rospy.get_rostime()

        # self._nearest_face_sub = rospy.Subscriber("/face_detector/nearest_face", FaceFound, self._nearest_face_cb)
        # self._stop_dist = 1.0

        # # self._base_cmd_pub = rospy.Publisher("/movo/base/follow_me/cmd_vel", Twist, queue_size = 1)
        # self._base_cmd_pub = rospy.Publisher("/movo/base/movo_closer/cmd_vel", Twist, queue_size = 1)
        #
        # self._base_cfg_pub = rospy.Publisher("/movo/gp_command", ConfigCmd, queue_size = 10)
        # self._base_cfg_msg = ConfigCmd()

        # is_sim = rospy.get_param("~sim", False)
        is_sim = False
        self._movo_base = MoveBaseActionClient(sim=is_sim, frame="odom")
        self._current_base_pos_x = 0.0
        self._current_base_pos_y = 0.0
        self._current_base_rot_z = 0.0

        target = Pose2D(x=0.0, y=0.0, theta=0.0)
        self._movo_base.goto(target)
        rospy.sleep(0.1)


        rospy.loginfo("Follow Me initialization finished")
        rospy.spin()


    """
    Move to nearest face.
    """
    def _nearest_face_cb(self, msg):

        print "pan_pose is ", msg.pan_pose, ", tilt_pose is ", msg.tilt_pose, ", dist of face is ", msg.face_dist


        # # enable movo base motion
        # if self._is_first_run:
        #     # mimic press joystick button 4 continuously for 0.1sec. Publish just one time with first run not working
        #     while rospy.get_rostime() - self._start_time < rospy.Duration(0.1):
        #         # make sure robot can move the base
        #         self._base_cfg_msg.gp_cmd = "GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE"
        #         self._base_cfg_msg.gp_param = TRACTOR_REQUEST
        #         self._base_cfg_msg.header.stamp = rospy.get_rostime()
        #         self._base_cfg_pub.publish(self._base_cfg_msg)
        #
        #     self._base_cfg_msg.gp_cmd = 'GENERAL_PURPOSE_CMD_NONE'
        #     self._base_cfg_msg.gp_param = 0
        #     self._base_cfg_msg.header.stamp = rospy.get_rostime()
        #     self._base_cfg_pub.publish(self._base_cfg_msg)
        #     self._is_first_run = False
        #     # self._base_cfg_pub.unregister()
        #
        # print "pan_pose is ", msg.pan_pose, ", tilt_pose is ", msg.tilt_pose, ", dist of face is ", msg.face_dist
        #
        # base_cmd_vel = Twist()
        #
        # base_cmd_vel.linear.z = 0.0
        # base_cmd_vel.angular.x = 0.0
        # base_cmd_vel.angular.y = 0.0
        #
        # # apply low pass filter
        # base_cmd_vel.linear.x = round(self._base_x_speed_filter.get_output(base_cmd_vel.linear.x, (rospy.get_rostime() - self._sampling_time).to_sec()), 3)
        # base_cmd_vel.linear.y = round(self._base_y_speed_filter.get_output(base_cmd_vel.linear.y, (rospy.get_rostime() - self._sampling_time).to_sec()), 3)
        # base_cmd_vel.angular.z = round(self._base_theta_z_speed_filter.get_output(base_cmd_vel.angular.z, (rospy.get_rostime() - self._sampling_time).to_sec()), 3)
        # self._sampling_time = rospy.get_rostime()
        #
        # return base_cmd_vel




if __name__ == "__main__":
    rospy.loginfo("start Move Closer")
    rospy.init_node("move_closer", log_level=rospy.DEBUG)
    # rospy.init_node("move_closer", log_level=rospy.INFO)

    MoveCloser()

