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
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from movo_msgs.msg import JacoCartesianVelocityCmd
from movo.system_defines import TRACTOR_REQUEST
from movo_msgs.msg import ConfigCmd
from movo_msgs.msg import JacoStatus


class LowPassFilter:
    def __init__(self, u0 = 0.0, y0 = 0.0, k = 10.0, Tconst = 1.0, Tsampling = 0.01):
        # initial input
        self._u = u0
        # initial output
        self._y = y0
        # filter gain
        self._k = k
        # setting time
        self._Tconst = Tconst
        # sampling time
        self._Tsampling = Tsampling

    # sampling time may change in ROS for each cycle
    def get_output(self, u, Tsampling):
        self._Tsampling = Tsampling

        # using forward Euler method on discrete transfer function:
        #                   (Tsamping/Tconst) * z^(-1)
        # G(z) = K * ----------------------------------------
        #              1 + (Tsampling/Tconst - 1) * z^(-1)
        #
        self._y = (1 - self._Tsampling/self._Tconst) * self._y + self._k * (self._Tsampling/self._Tconst) * u
        return self._y


class FollowMe:
    def __init__(self):
        dof = rospy.get_param('/init_robot/jaco_dof', '6dof')
        if dof == '6dof':
            self._dof = 6

            # below which considered as a level noise could reach, no external force applied to corresponding joint except gravity force.
            # values are based on test data when robot in DIFFERENT configurations, does not guarantee no external is applied for sure.
            # can be used as a "soft" "loose" "lower" boundary to detect external joint force
            self.angular_force_gravity_free_deadzone = [1.5, 1.5, 1.5, 0.5, 2.0, 0.5]

        elif dof == '7dof':
            self._dof = 7

            # TODO: find thresholds for 7DOF based on experiments
            # below which considered as a level noise could reach, no external force applied to corresponding joint except gravity force.
            # values are based on test data when robot in different configuration, does not guarantee no external is applied for sure.
            # can be used as a "soft" "loose" "lower" boundary to detect external joint force
            self.angular_force_gravity_free_deadzone = [10.0] * self._dof

        else:
            raise ValueError("Please check ros parameter /init_robot/jaco_dof, it should be either '6dof' or '7dof' ")

        # define the interpolation shape between force and speed of movo base
        # IMPORTANT: cartesian force range and offset are defined in the PRE-DEFINED follow-me-hand-pick-pose. They are not general, and could vary in large range for other arm configurations. If you define another follow-me-hand-pick-pose, make sure tune these values accordingly. eg: in the predefined pose, force_x is around +2.5, max applied force is 20, then range can be defined as [x+2.5 for x in [-20, 20].
        self._base_cartesian_force_x_range = [x + 2.5 for x in [-20.0, -13.0,  -7.0, 7.0, 13.0, 20.0] ]
        self._base_translation_speed_x_range = [-0.5, -0.3, 0.0, 0.0, 0.3, 0.5]
        self._base_cartesian_force_y_range = [x - 0.5 for x in [-20.0, -13.0,  -10.0, 10.0, 13.0, 20.0] ] # add offset without applied force
        self._base_translation_speed_y_range = [-0.4, -0.3, 0.0, 0.0, 0.3, 0.4]
        self._base_cartesian_torque_z_range = [x + 0.05 for x in [-5.0, -3.5, -3.0, 3.0, 3.5, 5.0] ]
        self._base_rotation_speed_z_range = [-0.8, -0.6, 0.0, 0.0, 0.6, 0.8]
        self._base_rotation_torque_threshold = min(map(abs, self._base_cartesian_torque_z_range))
        # "translation" or "rotation"
        self._base_motion_mode = ''

        self._Tsampling = 0.01
        self._sampling_time = rospy.get_rostime()
        self._base_x_speed_filter = LowPassFilter(k=1.0, Tconst=0.2, Tsampling=0.01)
        self._base_y_speed_filter = LowPassFilter(k=1.0, Tconst=0.2, Tsampling=0.01)
        self._base_theta_z_speed_filter = LowPassFilter(k=1.0, Tconst=0.1, Tsampling=0.01)


        self._is_first_run = True
        self._start_time = rospy.get_rostime()
        self._is_applied_force_valid = False

        self._right_cartesian_force_sub = rospy.Subscriber("/movo/right_arm/cartesianforce", JacoCartesianVelocityCmd, self._right_cartesian_force_cb)
        # self._right_cartesian_force_sub = rospy.Subscriber("/movo/right_arm/cartesianforce")
        self._base_force_msg = JacoCartesianVelocityCmd()
        self._base_force_msg.header.seq = 0
        self._base_force_msg.header.stamp = rospy.get_rostime()
        self._base_force_msg.header.frame_id = "base_link"

        self._teleop_control_mode_sub = rospy.Subscriber("movo/teleop/control_mode", String, self._teleop_control_mode_cb)
        # base_ctl, arm_ctl_right, arm_ctl_left, pan_tilt_ctl, estop, home_arms
        self._teleop_control_mode = 'base_ctl'
        self._teleop_control_mode_mutex = threading.RLock()

        self._angular_force_gravity_free_sub = rospy.Subscriber("/movo/right_arm/angularforce_gravityfree", JacoStatus, self._angular_force_gravity_free_cb)
        self._angular_force_gravity_free_mutex = threading.Lock()
        self._angular_force_gravity_free = [0.0] * self._dof

        """
        # # the transformation from _tf_update adds evident delay (1~10 seconds) to the computation of wrench in movo_base frame. thus abandoned
        # self._armbase_to_movobase_trans = numpy.zeros((3,1))
        # self._armbase_to_movobase_rot = numpy.eye(3)
        # self._armeef_to_movobase_trans = numpy.zeros((3,1))
        # self._armeef_to_movobase_rot = numpy.eye(3)
        # self._tf_listener = tf.TransformListener()
        # self._tf_mutex = threading.Lock()
        # # self._tf_listener_timer = rospy.Timer(0.01, self._tf_listener_timer_cb)
        # self._tf_thread = threading.Thread(target=self._tf_update)
        # self._tf_thread.start()
        """

        # for develop and debug purpose
        self._base_force_pub = rospy.Publisher("/movo/base/cartesianforce", JacoCartesianVelocityCmd, queue_size = 1)

        self._base_cmd_pub = rospy.Publisher("/movo/base/follow_me/cmd_vel", Twist, queue_size = 1)
        # self._base_cmd_pub = rospy.Publisher("/movo/base/follow_me/cmd_vel2", Twist, queue_size = 1)

        self._base_cfg_pub = rospy.Publisher("/movo/gp_command", ConfigCmd, queue_size = 10)
        self._base_cfg_msg = ConfigCmd()

        rospy.loginfo("Follow Me initialization finished")
        rospy.spin()


    def _teleop_control_mode_cb(self, msg):
        with self._teleop_control_mode_mutex:
            self._teleop_control_mode = msg.data


    def _angular_force_gravity_free_cb(self, msg):
        with self._angular_force_gravity_free_mutex:
            if msg.type == "angularforce_gravityfree":
                self._angular_force_gravity_free = msg.joint
                # if the torque on last joint is larger than all the others, or it is abnormally large consider want to switch to rotation mode
                if all(abs(self._angular_force_gravity_free[-1]) > abs(x) for x in self._angular_force_gravity_free[0:-1]) or abs(self._angular_force_gravity_free[-1]) > self._base_rotation_torque_threshold:
                    self._base_motion_mode = "rotation"
                else:
                    self._base_motion_mode = "translation"

    """
    # the transformation from _tf_update adds evident delay (1~10 seconds) to the computation of wrench in movo_base frame. thus abandoned
    def _tf_update(self):
        # rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                if self._tf_listener.frameExists("base_link") and self._tf_listener.frameExists("right_base_link"):
                    (trans, rot) = self._tf_listener.lookupTransform("/base_link", "/right_base_link", rospy.Time(0))
                    with self._tf_mutex:
                        self._armbase_to_movobase_trans = trans
                        self._armbase_to_movobase_rot = tf.transformations.quaternion_matrix(rot)[0:3, 0:3]
                    rospy.logdebug("right_base_link w.r.t. movobase: translation [%3.3f, %3.3f, %3.3f]", trans[0], trans[1], trans[2])
                else:
                    rospy.loginfo("Did not find frames /base_link or /right_base_link")


                if self._tf_listener.frameExists("base_link") and self._tf_listener.frameExists("right_ee_link"):
                    (trans, rot) = self._tf_listener.lookupTransform("/base_link", "/right_ee_link", rospy.Time(0))
                    with self._tf_mutex:
                        self._armeef_to_movobase_trans = trans
                        self._armeef_to_movobase_rot = tf.transformations.quaternion_matrix(rot)[0:3, 0:3]
                else:
                    rospy.loginfo("Did not find frames /base_link or /right_base_link")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Failed to get the transform in follow me demo")
                continue

            # rate.sleep()
    """


    def _right_cartesian_force_cb(self, msg):

        # enable movo base motion
        if self._is_first_run:
            # mimic press joystick button 4 continuously for 0.1sec. Publish just one time with first run not working
            while rospy.get_rostime() - self._start_time < rospy.Duration(0.1):
                # make sure robot can move the base
                self._base_cfg_msg.gp_cmd = "GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE"
                self._base_cfg_msg.gp_param = TRACTOR_REQUEST
                self._base_cfg_msg.header.stamp = rospy.get_rostime()
                self._base_cfg_pub.publish(self._base_cfg_msg)

            self._base_cfg_msg.gp_cmd = 'GENERAL_PURPOSE_CMD_NONE'
            self._base_cfg_msg.gp_param = 0
            self._base_cfg_msg.header.stamp = rospy.get_rostime()
            self._base_cfg_pub.publish(self._base_cfg_msg)
            self._is_first_run = False
            # self._base_cfg_pub.unregister()

        # check validation of applied external force
        self._is_applied_force_valid = True
        with self._teleop_control_mode_mutex:
            if self._teleop_control_mode in ["home_arms", "estop", "arm_ctl_right"]:
                rospy.logdebug("follow me will not be activated if corresponding arm control is enabled")
                self._is_applied_force_valid = False

        with self._angular_force_gravity_free_mutex:
            if all( [abs(x)<y for x,y in zip(self._angular_force_gravity_free, self.angular_force_gravity_free_deadzone)] ):
                rospy.logdebug("In each joint, the gravity-free torque is below noise threshold, consider as no force applied")
                self._is_applied_force_valid = False
            elif all( [abs(x)<y for x,y in zip(self._angular_force_gravity_free[self._dof-3:], self.angular_force_gravity_free_deadzone[self._dof-3:])] ):
                rospy.logdebug("The applied force is before the wrist, it will cause unexpected motion")
                self._is_applied_force_valid = False
            else:
                # rospy.logdebug("The applied force detected after robot wrist, follow me in process")
                pass

        """
        Transform force vector w.r.t. jaco arm frame to force vector w.r.t. movo_base frame.
        Use transformation matrix by tf could make it very general for more complex cases, eg: both frames are changing
        """
        # transform force applied on arm to force applied on Movo Base
        self._base_force_msg.header.stamp = rospy.get_rostime()
        self._base_force_msg.header.seq += 1

        # preceeding -1.0, expressively indicating the converting arm-counter-force to user-applied-force
        # IMPORTANT: the reference frame of force is the arm base, then transformed to movo base.
        self._base_force_msg.x = -1.0 * round(msg.z, 3)
        self._base_force_msg.y = -1.0 * round(-msg.x, 3)
        self._base_force_msg.z = -1.0 * round(-msg.y, 3)

        if abs(self._base_force_msg.z) >= abs(self._base_force_msg.x) and abs(self._base_force_msg.z) >= abs(self._base_force_msg.y):
            self._is_applied_force_valid = False

        # IMPORTANT: Based on tests, the reference frame of torque is another frame which is constant w.r.t. the end-effector frame.
        # TODO: the torque on arm end-effector could also be used for more comprehensive solution
        torque_temp_frame_x = -1.0 * round(msg.theta_x, 3)
        torque_temp_frame_y = -1.0 * round(msg.theta_y, 3)
        torque_temp_frame_z = -1.0 * round(msg.theta_z, 3)
        # transfer temp_torque_frame to end-effector frame
        torque_eef_frame_x = 1.0 * torque_temp_frame_z
        torque_eef_frame_y = -1.0 * torque_temp_frame_x
        torque_eef_frame_z = -1.0 * torque_temp_frame_y

        """
        # the transformation from _tf_update adds evident delay (1~10 seconds) to the computation of wrench in movo_base frame. thus abandonted
        # with self._tf_mutex:
            # movo_base_torque_z = Rotation_matrix_of_eef_frame_w.r.t._movo_base * Torque_eef_frame
            # base_force_wrench = self._armeef_to_movobase_rot.dot(numpy.array([torque_eef_frame_x, torque_eef_frame_y, torque_eef_frame_z]).reshape(3,1))
            # self._base_force_msg.theta_x = base_force_wrench[0]
            # self._base_force_msg.theta_y = base_force_wrench[1]
            # self._base_force_msg.theta_z = base_force_wrench[2]
            # rospy.logdebug("base_force_wrench is " + ", ".join("%3.3f"%x for x in base_force_wrench))
        """

        # use torque along gripper to control robot rotation
        self._base_force_msg.theta_z = torque_eef_frame_x
        # rospy.logdebug("torque in eef_frame is " + ", ".join("%3.3f" % x for x in [torque_eef_frame_x, torque_eef_frame_y, torque_eef_frame_z]))

        # debug info
        self._base_force_pub.publish(self._base_force_msg)

        # transform Movo base force to movo base motion
        base_cmd_vel = self._base_admittance_model()

        # publish base velocity command or reset base force for filter in next cycle
        if self._is_applied_force_valid:
            self._base_cmd_pub.publish(base_cmd_vel)
        else:
            self._base_force_msg.x = 0.0
            self._base_force_msg.y = 0.0
            self._base_force_msg.z = 0.0
            self._base_force_msg.theta_x = 0.0
            self._base_force_msg.theta_y = 0.0
            self._base_force_msg.theta_z = 0.0


    """
    Transform force command to motion command.
    """
    def _base_admittance_model(self):
        # TODO: more comprehensive algorithm could be developed here. Including virtual damping, virtual mass for the motion smoothness

        base_cmd_vel = Twist()
        with self._angular_force_gravity_free_mutex:
            if self._base_motion_mode == 'translation':
                # base_cmd_vel.linear.x = numpy.clip(trans_gain * self._base_force_msg.x, -self._base_translation_speed_max, self._base_translation_speed_max)
                # base_cmd_vel.linear.y = numpy.clip(trans_gain * self._base_force_msg.y, -self._base_translation_speed_max, self._base_translation_speed_max)
                base_cmd_vel.linear.x = round(numpy.interp(self._base_force_msg.x, self._base_cartesian_force_x_range, self._base_translation_speed_x_range), 3)
                base_cmd_vel.linear.y = round(numpy.interp(self._base_force_msg.y, self._base_cartesian_force_y_range, self._base_translation_speed_y_range), 3)
                base_cmd_vel.angular.z = 0.0
            elif self._base_motion_mode == 'rotation':
                base_cmd_vel.linear.x = 0.0
                base_cmd_vel.linear.y = 0.0
                # -1.0 to invert the rotation direction, based on test for user convenience in pre-defined arm pose
                # base_cmd_vel.angular.z = numpy.clip(rot_gain * self._base_force_msg.theta_z, -self._base_rotation_speed_max, self._base_rotation_speed_max)
                base_cmd_vel.angular.z = -1.0*round(numpy.interp(self._base_force_msg.theta_z, self._base_cartesian_torque_z_range, self._base_rotation_speed_z_range), 3)
            else:
                pass

        base_cmd_vel.linear.z = 0.0
        base_cmd_vel.angular.x = 0.0
        base_cmd_vel.angular.y = 0.0

        # apply low pass filter
        base_cmd_vel.linear.x = round(self._base_x_speed_filter.get_output(base_cmd_vel.linear.x, (rospy.get_rostime() - self._sampling_time).to_sec()), 3)
        base_cmd_vel.linear.y = round(self._base_y_speed_filter.get_output(base_cmd_vel.linear.y, (rospy.get_rostime() - self._sampling_time).to_sec()), 3)
        base_cmd_vel.angular.z = round(self._base_theta_z_speed_filter.get_output(base_cmd_vel.angular.z, (rospy.get_rostime() - self._sampling_time).to_sec()), 3)
        self._sampling_time = rospy.get_rostime()

        return base_cmd_vel


if __name__ == "__main__":
    rospy.loginfo("start Follow Me Demo")
    rospy.init_node("follow_me", log_level=rospy.DEBUG)
    # rospy.init_node("follow_me", log_level=rospy.INFO)
    rospy.loginfo("waiting for topic: /movo/right_arm/cartesianforce")
    rospy.wait_for_message("/movo/right_arm/cartesianforce", JacoCartesianVelocityCmd)

    FollowMe()

