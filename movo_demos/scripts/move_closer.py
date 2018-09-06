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
\Platform: Ubuntu 16.04 LTS / ROS Kinetic
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

        self._stop_dist = 1.0

        is_sim = rospy.get_param("~sim", False)
        self._movo_base = MoveBaseActionClient(sim=is_sim, frame="odom")

        self._nearest_face = Pose2D(x=0.0, y=0.0, theta=0.0)

        self._has_pose_target = False

        self._nearest_face_mutex = threading.Lock()

        self._move_base_thread   = threading.Thread(target = self._move_base_thread_cb)
        self._move_base_thread.start()

        self._nearest_face_sub = rospy.Subscriber("/face_detector/nearest_face", FaceFound, self._nearest_face_cb)

        rospy.loginfo("Follow Me initialization finished")
        rospy.spin()


    """
    Update base target according to found nearest face.
    """
    def _nearest_face_cb(self, msg):

        # run only once to update target pose
        if not self._has_pose_target:
            rospy.sleep(0.001)

            with self._nearest_face_mutex:
                self._nearest_face.x = msg.face_dist * numpy.cos(msg.tilt_pose) * numpy.cos(-msg.pan_pose)
                self._nearest_face.y = msg.face_dist * numpy.cos(msg.tilt_pose) * numpy.sin(-msg.pan_pose)
                # self._nearest_face.z = msg.face_dist * numpy.sin(msg.tilt_pose)
                self._nearest_face.theta = -msg.pan_pose

                self._has_pose_target = True
                rospy.loginfo("[pan_pose, tilt_pose, dist_of_face] is [%f, %f, %f] " %( msg.pan_pose, msg.tilt_pose, msg.face_dist) )
                self._nearest_face_sub.unregister()


    def _move_base_thread_cb(self):

        rate = rospy.Rate(10)
        while (not self._has_pose_target) and (not rospy.is_shutdown()):
            rospy.logdebug("wait for new target accepted " )
            rate.sleep()
            pass

        base_target = Pose2D(x=self._nearest_face.x - self._stop_dist * numpy.cos(self._nearest_face.theta),
                                   y=self._nearest_face.y - self._stop_dist * numpy.sin(self._nearest_face.theta),
                                   theta=self._nearest_face.theta)
        rospy.loginfo("send movo base to [x, y, theta] is [%f, %f, %f] "%(base_target.x, base_target.y, base_target.theta) )
        self._movo_base.goto(base_target)
        rospy.sleep(10.0)


if __name__ == "__main__":
    rospy.loginfo("start Move Closer")
    # rospy.init_node("move_closer", log_level=rospy.DEBUG)
    rospy.init_node("move_closer", log_level=rospy.INFO)

    MoveCloser()

