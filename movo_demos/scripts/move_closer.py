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

        self._stop_dist = 1.5

        is_sim = rospy.get_param("~sim", False)
        self._movo_base = MoveBaseActionClient(sim=is_sim, frame="odom")


        self._base_target = Pose2D(x=0.0, y=0.0, theta=0.0)
        self._movo_base.goto(self._base_target)
        rospy.sleep(1)


        self._is_base_rot_done = False
        self._is_base_forward_done = False
        self._is_accept_new_target = True

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
        print "pan_pose is ", msg.pan_pose, ", tilt_pose is ", msg.tilt_pose, ", dist of face is ", msg.face_dist

        # run only once to update target pose
        if self._is_accept_new_target:
            rospy.sleep(0.001)


            with self._nearest_face_mutex:
                self._base_target.x = msg.face_dist * numpy.cos(msg.tilt_pose) * numpy.cos(msg.pan_pose)
                self._base_target.y = msg.face_dist * numpy.cos(msg.tilt_pose) * numpy.sin(msg.pan_pose)

                self._base_target.x *= 0.5
                self._base_target.y *= 0.5
                self._base_target.theta = -msg.pan_pose

                self._is_accept_new_target = False
                rospy.loginfo(
                    "[x, y, theta] is [%f, %f, %f] " %(self._base_target.x, self._base_target.y, self._base_target.theta) )


    def _move_base_thread_cb(self):

        rate = rospy.Rate(10)
        # with self._nearest_face_mutex:
            # wait for new target accepted
        while (self._is_accept_new_target) and (not rospy.is_shutdown()):
            rospy.loginfo("wait for new target accepted " )
            rate.sleep()
            pass

        rospy.loginfo("[x, y, theta] is [%f, %f, %f] "%(self._base_target.x, self._base_target.y, self._base_target.theta) )
        self._movo_base.goto(self._base_target)


if __name__ == "__main__":
    rospy.loginfo("start Move Closer")
    # rospy.init_node("move_closer", log_level=rospy.DEBUG)
    rospy.init_node("move_closer", log_level=rospy.INFO)

    MoveCloser()

