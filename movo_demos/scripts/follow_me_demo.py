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

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from movo_msgs.msg import JacoCartesianVelocityCmd
from movo.system_defines import TRACTOR_REQUEST
from movo_msgs.msg import ConfigCmd
from movo_msgs.msg import JacoStatus

class FollowMe:
    def __init__(self):
        dof = rospy.get_param('/init_robot/jaco_dof', '6dof')
        if dof == '6dof':
            self._dof = 6

            # below which considered as a level noise could reach, no external force applied to corresponding joint except gravity force.
            # values are based on test data when robot in different configuration, does not guarantee no external is applied for sure.
            # can be used as a "soft" "loose" "lower" boundary to detect external joint force
            self.angular_force_gravity_free_deadzone = [5.7, 2.0, 0.5, 2.0, 1.5, 0.5]

            # above which considered surely external force applied to corresponding joints. However, below these values, it is also possible that external forces are applied
            # can be used as a "hard" "strict" "higher" boundary to detect external joint force
            self.angular_force_gravity_free_certainty = [6.0, 3.0, 3.0, 3.0, 2.0, 1.0]
        elif dof == '7dof':
            self._dof = 7

            # TODO: find thresholds for 7DOF based on experiments
            # below which considered as a level noise could reach, no external force applied to corresponding joint except gravity force.
            # values are based on test data when robot in different configuration, does not guarantee no external is applied for sure.
            # can be used as a "soft" "loose" "lower" boundary to detect external joint force
            self.angular_force_gravity_free_deadzone = [10.0] * self._dof

            # above which considered surely external force applied to corresponding joints. However, below these values, it is also possible that external forces are applied
            # can be used as a "hard" "strict" "higher" boundary to detect external joint force
            self.angular_force_gravity_free_certainty = [10.0] * self._dof
        else:
            raise ValueError("Please check ros parameter /init_robot/jaco_dof, it should be either '6dof' or '7dof' ")


        # below which cartesian force considered as zero, independent in each axis or each arm
        self.cartesian_force_deadzone = 10

        self._first_run = True
        self._timer_time = rospy.get_rostime()

        self._right_cartesian_force_sub = rospy.Subscriber("/movo/right_arm/cartesianforce", JacoCartesianVelocityCmd, self._right_cartesian_force_cb)
        # self._right_cartesian_force_sub = rospy.Subscriber("/movo/right_arm/cartesianforce")
        self._base_force_msg = JacoCartesianVelocityCmd()
        self._base_force_msg.header.seq = 0
        self._base_force_msg.header.stamp = rospy.get_rostime()
        self._base_force_msg.header.frame_id = "base_link"

        self._teleop_control_mode_sub = rospy.Subscriber("movo/teleop/control_mode", String, self._teleop_control_mode_cb)
        # base_ctl, arm_ctl_right, arm_ctl_left, pan_tilt_ctl, estop, home_arms
        self._teleop_control_mode = ''
        self._teleop_control_mode_mutex = threading.RLock()

        self._angular_force_gravity_free_sub = rospy.Subscriber("/movo/right_arm/angularforce_gravityfree", JacoStatus, self._angular_force_gravity_free_cb)
        self._angular_force_gravity_free_mutex = threading.Lock()
        self._angular_force_gravity_free = [0.0] * self._dof

        # for develop and debug purpose
        self._base_force_pub = rospy.Publisher("/movo/base/cartesianforce", JacoCartesianVelocityCmd, queue_size = 10)

        self._base_cmd_pub = rospy.Publisher("/movo/base/follow_me/cmd_vel", Twist, queue_size = 10)

        self._base_cfg_pub = rospy.Publisher("/movo/gp_command", ConfigCmd, queue_size = 10)
        self._base_cfg_msg = ConfigCmd()


        rospy.loginfo("Follow Me initialization finished")
        rospy.spin()


    """
    Transform force command to motion command.
    """
    def _base_admittance_model(self):
        """
        TODO: more comprehensive algorithm could be developed here. Including virtual damping, virtual mass for the motion smoothness
        :return:
        """
        # 1N corresponding to 0.001m per second
        gain = 0.01

        base_cmd_vel = Twist()
        base_cmd_vel.linear.x = gain * self._base_force_msg.x
        base_cmd_vel.linear.y = gain * self._base_force_msg.y
        base_cmd_vel.linear.z = 0.0
        base_cmd_vel.angular.x = 0.0
        base_cmd_vel.angular.y = 0.0
        base_cmd_vel.angular.z = 0.0

        return base_cmd_vel


    def _teleop_control_mode_cb(self, msg):
        with self._teleop_control_mode_mutex:
            self._teleop_control_mode = msg.data


    def _angular_force_gravity_free_cb(self, msg):
        with self._angular_force_gravity_free_mutex:
            if msg.type == "angularforce_gravityfree":
                self._angular_force_gravity_free = msg.joint


    def _right_cartesian_force_cb(self, msg):
        """
        Transform force vector w.r.t. jaco arm frame to force vector w.r.t. movo_base frame.
        Use transformation matrix by tf could make it very general for more complex cases, eg: both frames are changing
        """
        with self._teleop_control_mode_mutex:
            if self._teleop_control_mode in ["home_arms", "estop", "arm_ctl_right"]:
                rospy.logdebug("follow me will not be activated if corresponding arm control is enabled")
                return None

        with self._angular_force_gravity_free_mutex:
            if all( [x<y for x,y in zip(self._angular_force_gravity_free, self.angular_force_gravity_free_deadzone)] ):
                rospy.logdebug("In each joint, the gravity-free torque is below noise threshold, consider as no force applied")
                return None
            elif all( [x<y for x,y in zip(self._angular_force_gravity_free[self._dof-3:], self.angular_force_gravity_free_deadzone[self._dof-3:])] ):
                rospy.logdebug("The applied force is before the wrist, it will cause unexpected motion")
                return None
            else:
                # rospy.logdebug("The applied force detected after robot wrist, follow me in process")
                pass

        self._base_force_msg.header.stamp = rospy.get_rostime()
        self._base_force_msg.header.seq += 1

        # since transformation matrix between two frames are constant and simple
        # preceeding -1.0, expressively indicating the converting arm-counter-force to user-applied-force
        self._base_force_msg.x = -1.0 * msg.z
        self._base_force_msg.y = -1.0 * -msg.x
        self._base_force_msg.z = -1.0 * -msg.y
        self._base_force_msg.theta_x = -1.0 * msg.theta_z
        self._base_force_msg.theta_y = -1.0 * -msg.theta_x
        self._base_force_msg.theta_z = -1.0 * -msg.theta_y

        # debug info
        self._base_force_pub.publish(self._base_force_msg)

        # regulate base_force with dead zone
        if abs(self._base_force_msg.x) < self.cartesian_force_deadzone:
            self._base_force_msg.x = 0.0
        if abs(self._base_force_msg.y) < self.cartesian_force_deadzone:
            self._base_force_msg.y = 0.0
        if abs(self._base_force_msg.z) < self.cartesian_force_deadzone:
            self._base_force_msg.z = 0.0

        if self._first_run:
            # mimic press joystick button 4 continuously for 0.1sec. Publish just one time with first run not working
            while rospy.get_rostime() - self._timer_time < rospy.Duration(0.1):
                # make sure robot can move the base
                self._base_cfg_msg.gp_cmd = "GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE"
                self._base_cfg_msg.gp_param = TRACTOR_REQUEST
                self._base_cfg_msg.header.stamp = rospy.get_rostime()
                self._base_cfg_pub.publish(self._base_cfg_msg)
                # self._base_cfg_pub.unregister()

            self._base_cfg_msg.gp_cmd = 'GENERAL_PURPOSE_CMD_NONE'
            self._base_cfg_msg.gp_param = 0
            self._base_cfg_msg.header.stamp = rospy.get_rostime()
            self._base_cfg_pub.publish(self._base_cfg_msg)
            self._first_run = False

        # publish base velocity command
        self._base_cmd_pub.publish(self._base_admittance_model())


if __name__ == "__main__":
    rospy.loginfo("start Follow Me Demo")
    rospy.init_node("follow_me", log_level=rospy.INFO)
    rospy.loginfo("waiting for topic: /movo/right_arm/cartesianforce")
    rospy.wait_for_message("/movo/right_arm/cartesianforce", JacoCartesianVelocityCmd)

    FollowMe()

