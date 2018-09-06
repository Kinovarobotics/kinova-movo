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
import numpy as np
import threading
import roslib; roslib.load_manifest('people_msgs')
import rospy
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import JointState
from topic_tools.srv import MuxSelect, MuxSelectRequest

from movo.utils import *
from movo_msgs.msg import PanTiltCmd
from movo_msgs.msg import FaceFound
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
        rospy.on_shutdown(self._shutdown)

        self.last_tracking_time = rospy.get_time()

        # camera related parameters
        self.max_pan_view_angle = np.radians(30)
        self.max_tilt_view_angle = np.radians(20)
        self.max_dt_lag = 0.5

        self.list_faces = []
        self.nearest_face = Face()

        self.head_cmd_srv = rospy.ServiceProxy("/head_cmd_mux/select", MuxSelect)
        rospy.wait_for_service('/head_cmd_mux/select')
        head_cmd_request = MuxSelectRequest()
        head_cmd_request.topic = '/movo/head/face_tracking/cmd'
        self.head_cmd_srv.call(head_cmd_request)

        self.sync_head_pose_mutex = threading.Lock()

        self.face_detector_sub = rospy.Subscriber("/face_detector/faces_cloud", PointCloud, self._face_tracking)
        # self.face_detector_sub = rospy.Subscriber("/face_detector/people_tracker_measuremes_array", PositionMeasurementArray, self._face_tracking)
        self.head_joint_state_sub = rospy.Subscriber("/movo/head/joint_states", JointState, self._joint_state_cb)

        self.head_motion_pub = rospy.Publisher("/movo/head/face_tracking/cmd", PanTiltCmd, queue_size=10)
        self.head_cmd = PanTiltCmd()

        self.face_found_pub = rospy.Publisher("/face_detector/nearest_face", FaceFound, queue_size=10)
        self.face_found_msg = FaceFound()
        self.face_found_msg.header.seq = 0
        self.face_found_msg.header.stamp = rospy.get_rostime()
        self.face_found_msg.header.frame_id = "camera_color_optical_frame"

        # self.pantilt_vel_lim = rospy.get_param('~sim_teleop_pan_tilt_vel_limit', 0.524)
        self.pan_vel_lim = self.max_pan_view_angle # np.radians(30) = 0.524
        self.tilt_vel_lim = self.max_tilt_view_angle
        # pose increment inside deadzone, no need to send cmd
        self.pantilt_pose_deadzone = np.radians(2.0)

        # head searching face motion
        self.head_searching_wait_time = 5.0
        self.pan_searching_increment = np.radians(3.0)
        self.pan_searching_limit = np.radians(60.0)
        self.tilt_searching_increment = np.radians(10.0)
        self.tilt_searching_limit = np.radians(30.0)
        self.head_searching_thread = threading.Thread(target=self.head_searching_cb)
        self.head_searching_thread.start()

        # rate.sleep()
        rospy.loginfo("FaceTracking initialization")
        rospy.spin()


    def _joint_state_cb(self, msg):
        with self.sync_head_pose_mutex:
        # update current pan-tilt pose
            self.head_cmd.pan_cmd.pos_rad = msg.position[0]
            self.head_cmd.tilt_cmd.pos_rad = msg.position[1]
            self.head_cmd.pan_cmd.vel_rps = msg.velocity[0]
            self.head_cmd.tilt_cmd.vel_rps = msg.velocity[1]

        # only run at the start
        # self.head_joint_state_sub.unregister()


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


    def _tracking_motion_pub(self):
        rospy.logdebug("======================================")
        dt = min(rospy.get_time() - self.last_tracking_time, self.max_dt_lag)
        self.last_tracking_time = rospy.get_time()

        pan_cmd = np.arctan2(self.nearest_face.x, self.nearest_face.z)
        tilt_cmd = -1.0 * np.arctan2(self.nearest_face.y, np.linalg.norm(np.array([self.nearest_face.x, self.nearest_face.z])))
        rospy.logdebug("Camera view (raw data) [pan_angle tilt_angle] are [%f, %f] degrees \n", np.degrees(pan_cmd), np.degrees(tilt_cmd))
        pan_cmd = np.clip(pan_cmd, -1.0*self.max_pan_view_angle, self.max_pan_view_angle)
        tilt_cmd = np.clip(tilt_cmd, -1.0 * self.max_tilt_view_angle, self.max_tilt_view_angle)

        # only send cmd when increment is outside dead zone
        if ( (abs(pan_cmd) > self.pantilt_pose_deadzone) or (abs(tilt_cmd) > self.pantilt_pose_deadzone) ):
            pan_increment = np.clip(pan_cmd, -self.pan_vel_lim, self.pan_vel_lim) * dt
            tilt_increment = np.clip(tilt_cmd, -self.tilt_vel_lim, self.tilt_vel_lim) * dt
            rospy.logdebug("Increment in dt [%f seconds] of [pan tilt] are [%f, %f] degrees \n", dt, np.degrees(pan_increment),
                          np.degrees(tilt_increment))

            with self.sync_head_pose_mutex:
                self.head_cmd.pan_cmd.pos_rad += pan_increment
                self.head_cmd.tilt_cmd.pos_rad += tilt_increment
                rospy.logdebug("raw command for [pan tilt] are [%f, %f] degrees \n", np.degrees(self.head_cmd.pan_cmd.pos_rad), np.degrees(self.head_cmd.tilt_cmd.pos_rad))

                self.head_cmd.pan_cmd.pos_rad = np.clip(self.head_cmd.pan_cmd.pos_rad, np.radians(-90.0), np.radians(90.0))
                self.head_cmd.tilt_cmd.pos_rad = np.clip(self.head_cmd.tilt_cmd.pos_rad, np.radians(-45.0), np.radians(60.0))

                self.head_cmd.pan_cmd.vel_rps = self.pan_vel_lim
                self.head_cmd.tilt_cmd.vel_rps = self.tilt_vel_lim

                self.head_motion_pub.publish(self.head_cmd)
        else:
            self.face_found_msg.header.seq += 1
            self.face_found_msg.header.stamp = rospy.get_rostime()

            with self.sync_head_pose_mutex:
                self.face_found_msg.pan_pose = self.head_cmd.pan_cmd.pos_rad
                self.face_found_msg.tilt_pose = self.head_cmd.tilt_cmd.pos_rad
                self.face_found_msg.face_dist = self.nearest_face.z

            self.face_found_pub.publish(self.face_found_msg)


    def head_searching_cb(self):
        rate = rospy.Rate(10)
        # refresh each time pan motion reach its limit on both sides
        reach_pan_limit_time = rospy.get_time()
        # During this period, head has both pan and tilt motion. After then, only pan motion until reach pan limit
        tilt_at_pan_limit_duration = 1.0

        while not rospy.is_shutdown() :

            if (rospy.get_time() - self.last_tracking_time) <= self.head_searching_wait_time:
                rospy.logdebug("No face detected for %3.1f second. Will start to pan head to search faces after %3.ff second", (rospy.get_time() - self.last_tracking_time), self.head_searching_wait_time)
            else:
                with self.sync_head_pose_mutex:
                    # pan from -pan_searching_limit to pan_searching_limit degrees with error of step size, and pan position changes in each step
                    if(self.head_cmd.pan_cmd.pos_rad >= self.pan_searching_limit) and (self.pan_searching_increment >=0.0 ):
                        self.pan_searching_increment *= -1.0
                        reach_pan_limit_time = rospy.get_time()

                    if(self.head_cmd.pan_cmd.pos_rad <= -self.pan_searching_limit) and (self.pan_searching_increment <= 0.0):
                        self.pan_searching_increment *= -1.0
                        reach_pan_limit_time = rospy.get_time()

                    # tilt from -tilt_searching_limit to tilt_searching_limit degrees with error of step size, and tilt position only changes when reach limit
                    if(self.head_cmd.tilt_cmd.pos_rad >= self.tilt_searching_limit) and (self.tilt_searching_increment >=0.0 ):
                        self.tilt_searching_increment *= -1.0
                        rospy.logdebug('positive: self.tilt_searching_increment is %3.3f', self.tilt_searching_increment)

                    if(self.head_cmd.tilt_cmd.pos_rad <= -self.tilt_searching_limit) and (self.tilt_searching_increment <= 0.0):
                        self.tilt_searching_increment *= -1.0
                        rospy.logdebug('negative: self.tilt_searching_increment is %3.3f', self.tilt_searching_increment)


                    if rospy.get_time() - reach_pan_limit_time < tilt_at_pan_limit_duration:
                        self.head_cmd.tilt_cmd.pos_rad += self.tilt_searching_increment

                    self.head_cmd.pan_cmd.pos_rad += self.pan_searching_increment

                    # joint limit is set bigger than (searching_limit + increment)
                    self.head_cmd.pan_cmd.pos_rad = np.clip(self.head_cmd.pan_cmd.pos_rad, np.radians(-90.0), np.radians(90.0))
                    self.head_cmd.tilt_cmd.pos_rad = np.clip(self.head_cmd.tilt_cmd.pos_rad, np.radians(-45.0), np.radians(45.0))

                    self.head_cmd.pan_cmd.vel_rps = self.pan_vel_lim
                    self.head_cmd.tilt_cmd.vel_rps = self.tilt_vel_lim

                    self.head_motion_pub.publish(self.head_cmd)

            rate.sleep()


    # this call back function is trigged on each time detect a face.
    # Otherwise, subscriber do not hear message from the topic /face_detector/faces_cloud/
    def _face_tracking(self, msg):
        if self._find_nearest_face(msg):
            self._tracking_motion_pub()
        else:
            rospy.logdebug("detected face is not clear for face tracking")


    # set back to default head control channel
    def _shutdown(self):
        head_cmd_request = MuxSelectRequest()
        head_cmd_request.topic = '/movo/head/teleop/cmd'
        self.head_cmd_srv.call(head_cmd_request)


if __name__ == "__main__":
    rospy.loginfo("start the node")
    rospy.init_node("face_detection")
    rospy.wait_for_message("/movo/head/joint_states", JointState)

    # start face tracking
    FaceTracking()



