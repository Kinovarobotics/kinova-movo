"""--------------------------------------------------------------------
Copyright (c) 2017, Kinova Robotics inc.

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
 
 \file   jaco_action_client

 \brief  Action client for JACO JTAS

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import sys

from copy import copy

import rospy
import math
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

class JacoActionClient(object):
    def __init__(self, arm='right', dof=''):
        if ''==dof:
            rospy.logerr('DoF parameter needs to be set 6 or 7')
            return

        self._client = actionlib.SimpleActionClient(
            'movo/%s_arm_controller/follow_joint_trajectory'%arm,
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self.dof = dof
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(arm)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.velocities = [0.0] * len(self._goal.trajectory.joint_names)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def add_point_deg(self, joints_degree, time):
        self.add_point(map(math.radians, joints_degree), time)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time(0.0)
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, arm='right'):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        if('6dof'==self.dof):
            self._goal.trajectory.joint_names = ['%s_shoulder_pan_joint'%arm,
                                                 '%s_shoulder_lift_joint'%arm,
                                                 '%s_elbow_joint'%arm,
                                                 '%s_wrist_1_joint'%arm,
                                                 '%s_wrist_2_joint'%arm,
                                                 '%s_wrist_3_joint'%arm]
                                             
     	elif('7dof'==self.dof):
            self._goal.trajectory.joint_names = ['%s_shoulder_pan_joint'%arm,
                                                 '%s_shoulder_lift_joint'%arm,
                                                 '%s_arm_half_joint'%arm,
                                                 '%s_elbow_joint'%arm,
                                                 '%s_wrist_spherical_1_joint'%arm,
                                                 '%s_wrist_spherical_2_joint'%arm,
                                                 '%s_wrist_3_joint'%arm]

        else:
            rospy.logerr('DoF needs to be set 6 or 7')
