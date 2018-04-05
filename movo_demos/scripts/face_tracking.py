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

import roslib; roslib.load_manifest('people_msgs')
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import JointState

from movo.utils import *
from movo_msgs.msg import PanTiltCmd
from movo.movo_teleop_full_system import MovoTeleopFullSystem

from people_msgs.msg import PositionMeasurementArray


class Face:
    def __init__(self, detected_face = None):
        if(detected_face == None):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.dist = 0.0
        else:
            self.x = detected_face.x
            self.y = detected_face.y
            self.z = detected_face.z
            self.dist = np.linalg.norm(np.array([self.x, self.y, self.z]))

        self.id = 0 # not used. useful when subscribed to PositionMeasurementArray to match faces


class FaceTracking:
    def __init__(self):

        self.last_run_time = rospy.get_time()

        # camera related parameters
        self.max_pan_view_angle = np.radians(30)
        self.max_tilt_view_angle = np.radians(20)
        self.max_dt_lag = 0.5

        self.list_faces = []
        self.nearest_face = Face()

        self.face_detector_sub = rospy.Subscriber("/face_detector/faces_cloud", PointCloud, self._face_tracking)
        # self.face_detector_sub = rospy.Subscriber("/face_detector/people_tracker_measuremes_array", PositionMeasurementArray, self._face_tracking)
        self.head_joint_state_sub = rospy.Subscriber("/movo/head/joint_states", JointState, self._joint_state_cb)

        self.head_motion_pub = rospy.Publisher("/movo/head/cmd", PanTiltCmd, queue_size=10)
        self.head_cmd = PanTiltCmd()

        self.pantilt_vel_lim = rospy.get_param('~sim_teleop_pan_tilt_vel_limit', 0.524)
        # pose increment inside deadzone, no need to send cmd
        self.pantilt_pose_deadzone = np.radians(5.0)

        # rate.sleep()
        rospy.loginfo("FaceTracking initialization")
        rospy.spin()


    def _joint_state_cb(self, msg):
        # update current pan-tilt pose
        self.head_cmd.pan_cmd.pos_rad = msg.position[0]
        self.head_cmd.tilt_cmd.pos_rad = msg.position[1]
        self.head_cmd.pan_cmd.vel_rps = msg.velocity[0]
        self.head_cmd.tilt_cmd.vel_rps = msg.velocity[1]

        # only run at the start
        self.head_joint_state_sub.unregister()


    def _find_nearest_face(self, msg):
        # clear the list of founded faces in this moment
        del self.list_faces[:]

        # when detected face is bad, face_detector send msg with empty pose []
        if (len(msg.points) == 0):
            return False

        # valid face found
        else:
            for detected_face in msg.points:
                self.list_faces.append(Face(detected_face))

            # extract the face which is nearest to the camera frame
            dist_nearest_face = min(face.dist for face in self.list_faces)

            # idx_nearest_face is one element list
            idx_nearest_face = [index for index in range(len(self.list_faces)) if
                                self.list_faces[index].dist == dist_nearest_face]
            self.nearest_face = self.list_faces[idx_nearest_face[0]]
            return True


    def _head_motion_pub(self):
        rospy.loginfo("======================================")
        dt = min(rospy.get_time() - self.last_run_time, self.max_dt_lag)
        self.last_run_time = rospy.get_time()

        pan_cmd = np.arctan2(self.nearest_face.x, self.nearest_face.z)
        tilt_cmd = -1.0 * np.arctan2(self.nearest_face.y, np.linalg.norm(np.array([self.nearest_face.x, self.nearest_face.z])))
        rospy.loginfo("Camera view (raw data) [pan_angle tilt_angle] are [%f, %f] degrees \n", np.degrees(pan_cmd), np.degrees(tilt_cmd))
        pan_cmd = np.clip(pan_cmd, -1.0*self.max_pan_view_angle, self.max_pan_view_angle)
        tilt_cmd = np.clip(tilt_cmd, -1.0 * self.max_tilt_view_angle, self.max_tilt_view_angle)

        if ( (abs(pan_cmd) > self.pantilt_pose_deadzone) or (abs(tilt_cmd) > self.pantilt_pose_deadzone) ):
            # regulate velocity between 0 to 1 as joystick input
            # pan_vel_cmd = pan_cmd/abs(pan_cmd) * min(abs(pan_cmd)/self.pantilt_vel_lim, 1.0)
            # tilt_vel_cmd = tilt_cmd/abs(tilt_cmd) * min(abs(tilt_cmd)/self.pantilt_vel_lim, 1.0)

            # pan_increment = pan_vel_cmd * self.pantilt_vel_lim * dt
            # tilt_increment = tilt_vel_cmd * self.pantilt_vel_lim * dt
            pan_increment = np.clip(pan_cmd, -self.pantilt_vel_lim, self.pantilt_vel_lim) * dt
            tilt_increment = np.clip(tilt_cmd, -self.pantilt_vel_lim, self.pantilt_vel_lim) * dt
            rospy.loginfo("Increment in dt [%f seconds] of [pan tilt] are [%f, %f] degrees \n", dt, np.degrees(pan_increment),
                          np.degrees(tilt_increment))

            self.head_cmd.pan_cmd.pos_rad += pan_increment
            self.head_cmd.tilt_cmd.pos_rad += tilt_increment
            rospy.loginfo("raw command for [pan tilt] are [%f, %f] degrees \n", np.degrees(self.head_cmd.pan_cmd.pos_rad), np.degrees(self.head_cmd.tilt_cmd.pos_rad))

            self.head_cmd.pan_cmd.pos_rad = np.clip(self.head_cmd.pan_cmd.pos_rad, (-math.pi / 2.0), (math.pi / 2.0))
            self.head_cmd.tilt_cmd.pos_rad = np.clip(self.head_cmd.tilt_cmd.pos_rad, (-math.pi / 2.0), (math.pi / 2.0))

            self.head_cmd.pan_cmd.vel_rps = self.pantilt_vel_lim
            self.head_cmd.tilt_cmd.vel_rps = self.pantilt_vel_lim

        # only send cmd when increment is outside dead zone
        # if ( (abs(pan_cmd) > self.pantilt_pose_deadzone) or (abs(tilt_cmd) > self.pantilt_pose_deadzone) ):
            self.head_motion_pub.publish(self.head_cmd)


    # this call back function is trigged on each time detect a face.
    # Otherwise, subscriber do not hear message from the topic /face_detector/faces_cloud/
    def _face_tracking(self, msg):
        if self._find_nearest_face(msg):
            self._head_motion_pub()
        else:
            rospy.loginfo("detected face is not clear for face tracking")


if __name__ == "__main__":
    rospy.loginfo("start the node")
    rospy.init_node("face_detection")
    rospy.wait_for_message("/movo/head/joint_states", JointState)

    # disable head pan-tilt motion input from joystick
    joystick_telelop = MovoTeleopFullSystem()
    joystick_telelop.run_pan_tilt_ctl = False

    # start face tracking
    FaceTracking()



