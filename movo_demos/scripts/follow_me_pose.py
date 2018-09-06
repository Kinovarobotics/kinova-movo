#!/usr/bin/env python
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

 \file   follow_me_pose

 \Author Longfei Zhao

 \Platform: Ubuntu 16.04 LTS / ROS Kinetic
--------------------------------------------------------------------"""
import numpy as np
import rospy
import sys
from std_msgs.msg import String
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes
from movo_action_clients.gripper_action_client import GripperActionClient

if __name__ == "__main__":
    rospy.init_node('follow_me_pose')

    voice_pub = rospy.Publisher("/movo/voice/text", String, queue_size=1, latch=True)
    voice_cmd = String()
    voice_cmd.data = "Do you want to dance with me?"
    # voice_cmd.data = "Long fei, stop wasting time on me. It will not work, unless you say I like movo."
    voice_pub.publish(voice_cmd)

    dof = rospy.get_param('~jaco_dof')

    rmove_group = MoveGroupInterface("right_arm", "base_link")
    lmove_group = MoveGroupInterface("left_arm", "base_link")
    lmove_group.setPlannerId("RRTConnectkConfigDefault")
    rmove_group.setPlannerId("RRTConnectkConfigDefault")
    lgripper = GripperActionClient('left')
    rgripper = GripperActionClient('right')

    if "6dof" == dof:
        right_arm_joints = ["right_shoulder_pan_joint",
                              "right_shoulder_lift_joint",
                              "right_elbow_joint",
                              "right_wrist_1_joint",
                              "right_wrist_2_joint",
                              "right_wrist_3_joint"]
        left_arm_joints = ["left_shoulder_pan_joint",
                             "left_shoulder_lift_joint",
                             "left_elbow_joint",
                             "left_wrist_1_joint",
                             "left_wrist_2_joint",
                             "left_wrist_3_joint"]
        rarm_pick = [np.radians(x) for x in [-75.0, -15.0, -84.0, 0.0, -120.0, 30.0]]
        larm_pick = [np.radians(x) for x in [84.8, 84.8, 160.4, 0.0, 0.0, -90.0]]

    elif "7dof" == dof:
        right_arm_joints = ["right_shoulder_pan_joint",
                                  "right_shoulder_lift_joint",
                                  "right_arm_half_joint",
                                  "right_elbow_joint",
                                  "right_wrist_spherical_1_joint",
                                  "right_wrist_spherical_2_joint",
                                  "right_wrist_3_joint"]
        left_arm_joints = ["left_shoulder_pan_joint",
                                 "left_shoulder_lift_joint",
                                 "left_arm_half_joint",
                                 "left_elbow_joint",
                                 "left_wrist_spherical_1_joint",
                                 "left_wrist_spherical_2_joint",
                                 "left_wrist_3_joint"]
        larm_pick = [2.6, -2.0, 0.0, -2.0, 0.0, 0.0, 1.0]
        rarm_pick = [-2.6, 2.0, 0.0, 2.0, 0.0, 0.0, -1.0]
        rospy.logwarn("follow_me pose for 7dof is not defined yet.")
    else:
        rospy.logerr("DoF needs to be set 6 or 7, aborting demo")
        sys.exit()

    # This is not simulation so need to adjust gripper parameters
    gripper_closed = 0.01
    gripper_open = 0.165

    success = False
    while not rospy.is_shutdown() and not success:
        l_result = lmove_group.moveToJointPosition(left_arm_joints, larm_pick, 0.05, planning_time=120.0)
        r_result = rmove_group.moveToJointPosition(right_arm_joints, rarm_pick, 0.05, planning_time=120.0)
        if (r_result.error_code.val == MoveItErrorCodes.SUCCESS) and (l_result.error_code.val == MoveItErrorCodes.SUCCESS):
            success = True
    lgripper.command(gripper_closed)
    rgripper.command(gripper_closed)

    rospy.spin()
