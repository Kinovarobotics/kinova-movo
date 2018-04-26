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

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import numpy as np
import rospy
from std_msgs.msg import String
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes
from movo_action_clients.gripper_action_client import GripperActionClient

if __name__ == "__main__":
    rospy.init_node('follow_me_pose')

    voice_pub = rospy.Publisher("/movo/voice/text", String, queue_size=1, latch=True)
    voice_cmd = String()
    voice_cmd.data = "Do you want to dance with me?"
    voice_pub.publish(voice_cmd)

    move_group = MoveGroupInterface("upper_body", "base_link")
    move_group.setPlannerId("RRTConnectkConfigDefault")
    lgripper = GripperActionClient('left')
    rgripper = GripperActionClient('right')

    dof = rospy.get_param('~jaco_dof')

    if "6dof" == dof:
        upper_body_joints = ["right_shoulder_pan_joint",
                             "right_shoulder_lift_joint",
                             "right_elbow_joint",
                             "right_wrist_1_joint",
                             "right_wrist_2_joint",
                             "right_wrist_3_joint",
                             "left_shoulder_pan_joint",
                             "left_shoulder_lift_joint",
                             "left_elbow_joint",
                             "left_wrist_1_joint",
                             "left_wrist_2_joint",
                             "left_wrist_3_joint",
                             "linear_joint",
                             "pan_joint",
                             "tilt_joint"]
        right_arm = [np.radians(x) for x in [-75.0, -15.0, -84.0, 0.0, -120.0, 30.0]]
        left_arm = [np.radians(x) for x in [84.8, 84.8, 160.4, 0.0, 0.0, -90.0]]
        followme_pose = right_arm + left_arm + [0.15, 0.0, 0.0]

    if "7dof" == dof:
        upper_body_joints = ["right_shoulder_pan_joint",
                             "right_shoulder_lift_joint",
                             "right_arm_half_joint",
                             "right_elbow_joint",
                             "right_wrist_spherical_1_joint",
                             "right_wrist_spherical_2_joint",
                             "right_wrist_3_joint",
                             "left_shoulder_pan_joint",
                             "left_shoulder_lift_joint",
                             "left_arm_half_joint",
                             "left_elbow_joint",
                             "left_wrist_spherical_1_joint",
                             "left_wrist_spherical_2_joint",
                             "left_wrist_3_joint",
                             "linear_joint",
                             "pan_joint",
                             "tilt_joint"]

        followme_pose = [-1.5, -1.6, 0.4, -2.7, 0.0, 0.5, -1.7, 1.5, 1.6, -0.4, 2.7, 0.0, -0.5, 1.7, 0.04, 0, 0]
        rospy.logwarn("follow_me pose for 7dof is not defined yet.")


    # This is not simulation so need to adjust gripper parameters
    gripper_closed = 0.01
    gripper_open = 0.165

    success = False
    while not rospy.is_shutdown() and not success:
        result = move_group.moveToJointPosition(upper_body_joints, followme_pose, 0.05)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            success = True
    lgripper.command(gripper_closed)
    rgripper.command(gripper_closed)

    rospy.spin()
