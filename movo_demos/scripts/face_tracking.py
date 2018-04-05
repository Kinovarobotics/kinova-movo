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

import roslib; roslib.load_manifest('face_detector')
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import JointState

from movo.utils import *
from movo_msgs.msg import PanTiltCmd
from movo_action_clients.head_action_client import HeadActionClient
from movo.movo_teleop_full_system import MovoTeleopFullSystem

from people_msgs.msg import PositionMeasurementArray
import face_detector.msg

from operator import attrgetter # min of list of class attributes

class Face:
    def __init__(self, detected_face = None):
        if(detected_face == None):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.dist = np.linalg.norm(np.array([self.x, self.y, self.z]))
        else:
            self.x = detected_face.x
            self.y = detected_face.y
            self.z = detected_face.z
            self.dist = np.linalg.norm(np.array([self.x, self.y, self.z]))
        # self.valid = False
        self.id = 0 # useful when subscribed to PositionMeasurementArray

    def update(self, detected_face):
        # if detected_face.id == self.id
        self.x = detected_face.x
        self.y = detected_face.y
        self.z = detected_face.z
        self.dist = np.linalg.norm(np.array([self.x, self.y, self.z]))

    def update_by_xyz(self, x, y, z):
        # if detected_face.id == self.id
        self.x = x
        self.y = y
        self.z = z
        self.dist = np.linalg.norm(np.array([self.x, self.y, self.z]))


class FaceTracking:
    def __init__(self):

        self.last_run_time = rospy.get_time()

        # camera related parameters
        self.max_pan_view_angle = np.radians(30)
        self.max_tilt_view_angle = np.radians(20)
        self.max_dt_lag = 0.5

        self.list_faces = []
        self.nearest_face = Face()
        self.is_face_detected = False

        # just for testing
        self.fakeface = Face()
        self.fakeface.update_by_xyz(1, 2, 3)

        # # set running rate as 0.1s for face tracking.
        # rate = rospy.Rate(0.1)

        self.face_detector_sub = rospy.Subscriber("/face_detector/faces_cloud", PointCloud, self._face_tracking)
        # self.face_detector_sub = rospy.Subscriber("/face_detector/people_tracker_measuremes_array", PositionMeasurementArray, self._face_tracking)

        self.head_motion_pub = rospy.Publisher("/movo/head/cmd", PanTiltCmd, queue_size=10)
        self.head_cmd = PanTiltCmd()

        self.movo_head_action = HeadActionClient()

        self.pantilt_vel_lim = rospy.get_param('~sim_teleop_pan_tilt_vel_limit', 0.524)

        # rate.sleep()
        rospy.loginfo("FaceTracking initialization")
        rospy.spin()

    def _find_nearest_face(self, msg):
        # clear the list of founded faces in this moment
        del self.list_faces[:]

        # when detected face is bad, face_detector send msg with empty pose []
        if (len(msg.points) == 0):
            rospy.loginfo("no valid face detected")
            return False

        # valid face found
        else:
            for detected_face in msg.points:
                self.list_faces.append(Face(detected_face))

            # extract the face which is nearest to the camera frame
            # dist_nearest_face = min(self.list_faces, key=lambda x: x.dist).dist
            dist_nearest_face = min(face.dist for face in self.list_faces)

            # idx_nearest_face is one element list
            idx_nearest_face = [index for index in range(len(self.list_faces)) if
                                self.list_faces[index].dist == dist_nearest_face]
            self.nearest_face = self.list_faces[idx_nearest_face[0]]
            return True


    def _head_motion_action(self, max_pan_vel, max_tilt_vel):
        pan_cmd = np.arctan2(self.nearest_face.x, self.nearest_face.z)
        tilt_cmd = -1.0 * np.arctan2(self.nearest_face.y, np.linalg.norm(np.array([self.nearest_face.x, self.nearest_face.z])))

        # assume movo can run face detection at a rate of 2hz (based on real time topic hz)
        face_detector_rate = 2

        # rectify the pan-tilt cmd (increment) of each loop based on maximum velicity and refreshing rate
        pan_cmd_rect = min(pan_cmd, max_pan_vel/face_detector_rate)
        tilt_cmd_rect = min(tilt_cmd, max_pan_vel/face_detector_rate)

        tmp_head = rospy.wait_for_message("/movo/head/joint_states", JointState)
        current_angles = tmp_head.position

        # clear trajectory list and add current pose of pan tilt
        self.movo_head_action.clear()
        time_from_start = 0.0
        self.movo_head_action.add_point(list(current_angles), 0.0)

        # take the maximum time from pan motion and tilt motion
        time_duration = max(pan_cmd_rect/max_pan_vel, tilt_cmd_rect/max_pan_vel)
        self.movo_head_action.add_point([pan_cmd_rect, tilt_cmd_rect], time_from_start + time_duration)

        info = "raw head motion cmd [pan, tilt] is trun " + ("right" if pan_cmd_rect > 0 else "left: ") + \
               str(round(abs(np.degrees(pan_cmd_rect)), 1)) + " degree, turn " + ("up" if tilt_cmd_rect > 0 else "down ") +\
               str(round(abs(np.degrees(tilt_cmd_rect)), 1)) + " degree"
        rospy.loginfo(info)

        self.movo_head_action.start()
        self.movo_head_action.wait(time_from_start + time_duration)


    def _head_motion_pub(self):
        rospy.loginfo("======================================")
        dt = min(rospy.get_time() - self.last_run_time, self.max_dt_lag)
        self.last_run_time = rospy.get_time()

        if self.is_face_detected:
            pan_cmd = np.arctan2(self.nearest_face.x, self.nearest_face.z)
            tilt_cmd = -1.0 * np.arctan2(self.nearest_face.y, np.linalg.norm(np.array([self.nearest_face.x, self.nearest_face.z])))
            rospy.loginfo("Camera view (raw data) [pan_angle tilt_angle] are [%f, %f] degrees \n", np.degrees(pan_cmd), np.degrees(tilt_cmd))
            pan_cmd = np.clip(pan_cmd, -1.0*self.max_pan_view_angle, self.max_pan_view_angle)
            tilt_cmd = np.clip(tilt_cmd, -1.0 * self.max_tilt_view_angle, self.max_tilt_view_angle)

            # regulate velocity between 0 to 1 as joystick input
            pan_vel_cmd = pan_cmd/abs(pan_cmd) * min(abs(pan_cmd)/self.pantilt_vel_lim, 1.0)
            tilt_vel_cmd = tilt_cmd/abs(tilt_cmd) * min(abs(tilt_cmd)/self.pantilt_vel_lim, 1.0)
        else:
            pan_cmd = 0.0
            tilt_cmd = 0.0
            pan_vel_cmd = 0.0
            tilt_vel_cmd = 0.0

        pan_increment = pan_vel_cmd * self.pantilt_vel_lim * dt
        tilt_increment = tilt_vel_cmd * self.pantilt_vel_lim * dt
        rospy.loginfo("Increment in dt [%f seconds] of [pan tilt] are [%f, %f] degrees \n", dt, np.degrees(pan_increment),
                      np.degrees(tilt_increment))

        self.head_cmd.pan_cmd.pos_rad += pan_increment
        self.head_cmd.tilt_cmd.pos_rad += tilt_increment

        info = "head motion increment [pan, tilt] is trun " + ("right: " if pan_increment > 0 else "left: ") + \
               str(round(abs(np.degrees(pan_increment)), 1)) + " degree, turn " + ("up: " if tilt_increment > 0 else "down: ") + \
               str(round(abs(np.degrees(tilt_increment)), 1)) + " degree. "
        rospy.loginfo(info)

        rospy.loginfo("raw command for [pan tilt] are [%f, %f] degrees \n", np.degrees(self.head_cmd.pan_cmd.pos_rad), np.degrees(self.head_cmd.tilt_cmd.pos_rad))

        self.head_cmd.pan_cmd.pos_rad = limit_f(self.head_cmd.pan_cmd.pos_rad, (math.pi / 2.0))
        self.head_cmd.tilt_cmd.pos_rad = limit_f(self.head_cmd.tilt_cmd.pos_rad, (math.pi / 2.0))

        self.head_cmd.pan_cmd.vel_rps = 50.0 * (math.pi / 180.0)
        self.head_cmd.tilt_cmd.vel_rps = 50.0 * (math.pi / 180.0)

        self.head_motion_pub.publish(self.head_cmd)


    # this call back function is trigged on each time detect a face.
    # Otherwise, subscriber do not hear message from the topic /face_detector/faces_cloud/
    def _face_tracking(self, msg):
        self.is_face_detected = self._find_nearest_face(msg)
        # self._head_motion_action(0.2, 0.2)
        self._head_motion_pub()


if __name__ == "__main__":
    rospy.loginfo("start the node")
    rospy.init_node("face_detection")
    rospy.wait_for_message("/movo/head/joint_states", JointState)

    # disable head pan-tilt motion input from joystick
    joystick_telelop = MovoTeleopFullSystem()
    joystick_telelop.run_pan_tilt_ctl = False

    # start face tracking
    FaceTracking()



