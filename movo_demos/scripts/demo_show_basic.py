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

\Author Longfei Zhao
\brief  Demo prepared for IROS 2017 in Vancouver
\Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""

import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from movo_action_clients.head_action_client import HeadActionClient
from movo.movo_voice import MovoVoice
from movo_action_clients.jaco_action_client import JacoActionClient
from movo_action_clients.gripper_action_client import GripperActionClient
from movo_action_clients.torso_action_client import TorsoActionClient
from movo_action_clients.move_base_action_client import MoveBaseActionClient
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

import datetime as dt

def say(_pub, _data):
    voice_cmd = String()
    voice_cmd.data = _data
    _pub.publish(voice_cmd)

if __name__ == "__main__":

    process_start_time = dt.datetime.now()
    rospy.init_node("demo_show_basic")
    dof = rospy.get_param('~jaco_dof')
    sim = rospy.get_param("~sim", False)
    if (sim):
        rospy.wait_for_message('/sim_initialized',Bool)

    movo_head = HeadActionClient()
    movo_base = MoveBaseActionClient(sim=sim, frame="odom")
    movo_larm = JacoActionClient(arm='left', dof=dof)
    movo_rarm = JacoActionClient(arm='right', dof=dof)
    movo_lfinger = GripperActionClient('left')
    movo_rfinger = GripperActionClient('right')
    movo_torsor = TorsoActionClient()
    Publisher = rospy.Publisher("/movo/voice/text", String, queue_size=1, latch=True)
    rospy.sleep(2)

    if "6dof" == dof:
        # arm position definition
        larm_home_deg = [85, 13, 122.5, 120, -83, -75]
        rarm_home_deg = [-1.0 * x for x in larm_home_deg]

        larm_greet_deg = [125, -30, 45, 120, 0, -170] # [2.18, -0.52, 0.79, 2.09, 0, -2.97]
        rarm_greet_deg = [-1.0 * x for x in larm_greet_deg]

        larm_traj_side_deg = [179, -90, 0, 0, 0, -179] # [3.12, -1.57, 0, 0, 0, -3.12]
        rarm_traj_side_deg = [-1.0 * x for x in larm_traj_side_deg]

        larm_traj_down_deg = [270, -45, -45, 90, 0, -179]  # [4.71, -0.79, -0.79, 1.57, -3.12]
        rarm_traj_down_deg = [-1.0 * x for x in larm_traj_down_deg]

        larm_kiss_close_deg = [34, 45, 130, 90, -45, 0] # [0.59, 0.79, 2.27,1.57, -0.79, 0]
        rarm_kiss_close_deg = [-1.0 * x for x in larm_kiss_close_deg]

        larm_kiss_open_deg = [45, 45, 20, 125, -15, -15] # [0.79, 0.79, 0.35, 2.18, -0.26, -0.26]
        rarm_kiss_open_deg = [-1.0 * x for x in larm_kiss_open_deg]

        larm_tuck_deg = [90, 90, 160, 120, 0, 0] # [1.57, 1.57, 2.79, 2.09, 0, 0]
        rarm_tuck_deg = [-1.0 * x for x in larm_tuck_deg]

        larm_tuck_buff_deg = [80, 80, 150, 140, -20, 0] # [1.4, 1.4, 2.62, 2.44, -0.35, 0]
        rarm_tuck_buff_deg = [-1.0 * x for x in larm_tuck_buff_deg]

    if "7dof" == dof:
        # arm position definition
        larm_home_deg = [85, 12, 9, 115, -115, 71, 63]
        rarm_home_deg = [-1.0 * x for x in larm_home_deg]

        larm_greet_deg = [125, -49, 45, 26, 0, 0, -75]   # [2.17, -0.85, 0.82, 0.46, 0, 0, -1.3]
        rarm_greet_deg = [-1.0 * x for x in larm_greet_deg]

        larm_traj_side_deg = [172, -64, 76, -6, 0, 0, -75]  # [3.0, -1.12, 1.33, -0.1, 0, 0, -1.3]
        rarm_traj_side_deg = [-1.0 * x for x in larm_traj_side_deg]

        larm_traj_down_deg = [93, 53, 0, 52, 0, 18, -85] # [1.63, 0.92, 0, 0.90, 0, 0.31, -1.48]
        rarm_traj_down_deg = [-1.0 * x for x in larm_traj_down_deg]

        larm_kiss_close_deg = [43, 46, 17, 140, 14, -36, 97]  # [0.75, 0.8, 0.3, 2.45, 0.25, -0.63, 1.7]
        rarm_kiss_close_deg = [-1.0 * x for x in larm_kiss_close_deg]

        larm_kiss_open_deg = [47, 40, 0, 18, 0, -14, 99] # [0.82, 0.7, 0.0, 0.32, 0.0, -0.24, 1.73]
        rarm_kiss_open_deg = [-1.0 * x for x in larm_kiss_open_deg]

        larm_tuck_deg = [92, 86, -23, 155, 0, -29, 97]
        rarm_tuck_deg = [-1.0 * x for x in larm_tuck_deg]

        larm_tuck_buff_deg = [84, 75, -6, 148, 31, -7, 0] # [1.46, 1.30, -0.11, 2.58, 0.54, -0.13, 0]
        rarm_tuck_buff_deg = [-1.0 * x for x in larm_tuck_buff_deg]

    """
    1. Greeting words
    """
    say(Publisher, "Hello there. My name is MOVO. "
                   "Welcome to IROS. You can get to know me, and all I can do, over the next minute and a half. ")
    rospy.sleep(1)
    movo_lfinger.command(0.165)
    movo_rfinger.command(0.165)
    movo_lfinger.wait(3)
    movo_rfinger.wait(3)
    movo_larm.clear('left')
    movo_rarm.clear('right')
    tmp_left = rospy.wait_for_message("/movo/left_arm/joint_states", JointState)
    current_larm_pos = list(tmp_left.position)
    tmp_right = rospy.wait_for_message("/movo/right_arm/joint_states", JointState)
    current_rarm_pos = list(tmp_right.position)
    from_start_time = 0.0
    movo_larm.add_point(current_larm_pos, from_start_time)
    movo_rarm.add_point(current_rarm_pos, from_start_time)
    from_start_time += 5
    movo_larm.add_point_deg(larm_greet_deg, from_start_time)
    movo_rarm.add_point_deg(rarm_greet_deg, from_start_time)
    from_start_time += 5
    movo_larm.add_point_deg(larm_home_deg, from_start_time)
    movo_rarm.add_point_deg(rarm_home_deg, from_start_time)
    # start motion
    movo_larm.start()
    movo_rarm.start()
    movo_larm.wait(from_start_time+2)
    movo_rarm.wait(from_start_time+2)
    print "Movo Completing Greeting Task!"
    # """
    # 2. Feature Motion
    # """
    # Movo base motion
    say(Publisher, "I can move all over the place. Forward... and backward... and to the side. "
                   "Do I look like Michael Jackson? ")
    target = Pose2D(x=0.2, y=0.0, theta=0.0)
    movo_base.goto(target)

    target = Pose2D(x=0.0, y=0.0, theta=0.0)
    movo_base.goto(target)
    rospy.sleep(1)

    target = Pose2D(x=0.0, y=0.3, theta=0.0)
    movo_base.goto(target)

    rospy.sleep(1)
    target = Pose2D(x=0.0, y=0.0, theta=0.0)
    movo_base.goto(target)
    rospy.sleep(1)

    say(Publisher, "I can even do a pirouette. ")
    target = Pose2D(x=0.0, y=0.0, theta=-0.52)
    movo_base.goto(target)

    rospy.sleep(1)
    target = Pose2D(x=0.0, y=0.0, theta=0.52)
    movo_base.goto(target)

    rospy.sleep(1)
    target = Pose2D(x=0.0, y=0.0, theta=0.0)
    movo_base.goto(target)
    print "Movo Completing Base Motion"

    # Torso motion
    say(Publisher, "There are certain advantages to being a robot. We do not have to worry about being too tall or too short. "
                   "Just right for whatever we need to do. ")
    movo_torsor.clear()
    temp_torso = rospy.wait_for_message("/movo/linear_actuator/joint_states", JointState)
    current_torso_pos = list(temp_torso.position)
    movo_torsor.add_point(current_torso_pos, 0.0)
    # movo_torsor.add_point([0.45], 4)
    movo_torsor.add_point([0.0], 6)
    movo_torsor.add_point([0.10], 12)
    movo_torsor.start()
    movo_torsor.wait(14.0)
    print "Movo Completing Torso Motion"

    # Arm motion
    if '6dof' == dof:
        say(Publisher, "Thanks to six degrees of freedom, I can move my arms in many ways. "
                       "It is very helpful for all sorts of tasks. And I run on open source, open architecture. I am ROS enabled. "
                       "Please join my BETA program and help me to be the best I-can be. ")
    if '7dof' == dof:
        say(Publisher, "Thanks to seven degrees of freedom, I can move my arms in many ways. "
                       "It is very helpful for all sorts of tasks. And I run on open source, open architecture. I am ROS enabled. "
                       "Please join my BETA program and help me to be the best I-can be. ")

    movo_larm.clear('left')
    movo_rarm.clear('right')
    tmp_left = rospy.wait_for_message("/movo/left_arm/joint_states", JointState)
    tmp_right = rospy.wait_for_message("/movo/right_arm/joint_states", JointState)
    current_larm_pos = list(tmp_left.position)
    current_rarm_pos = list(tmp_right.position)

    from_start_time = 0.0
    movo_larm.add_point(current_larm_pos, from_start_time)
    movo_rarm.add_point(current_rarm_pos, from_start_time)

    from_start_time += 6
    movo_larm.add_point_deg(larm_traj_side_deg, from_start_time)
    movo_rarm.add_point_deg(rarm_traj_side_deg, from_start_time)

    from_start_time += 5
    movo_larm.add_point_deg(larm_traj_down_deg, from_start_time)
    movo_rarm.add_point_deg(rarm_traj_down_deg, from_start_time)

    from_start_time += 8
    movo_larm.add_point_deg(larm_home_deg, from_start_time)
    movo_rarm.add_point_deg(rarm_home_deg, from_start_time)

    movo_rarm.start()
    movo_larm.start()
    movo_larm.wait(from_start_time+2)
    movo_rarm.wait(from_start_time+2)
    rospy.sleep(1)


    # Finger motion
    say(Publisher, "I have three fingers--they are designed to help me perform thousands of tasks")
    movo_lfinger.command(0.0)
    movo_rfinger.command(0.0)
    movo_lfinger.wait(3)
    movo_rfinger.wait(3)

    movo_lfinger.command(0.165)
    movo_rfinger.command(0.165)
    movo_lfinger.wait(3)
    movo_rfinger.wait(3)

    movo_lfinger.command(0.085)
    movo_rfinger.command(0.085)
    movo_lfinger.wait(3)
    movo_rfinger.wait(3)
    rospy.sleep(2)

    # Head motion
    tmp_head = rospy.wait_for_message("/movo/head/joint_states", JointState)
    current_angles= tmp_head.position

    movo_head.clear()
    time_from_start = 0.0
    movo_head.add_point(list(current_angles), 0.0)
    time_from_start += 2
    movo_head.add_point([0.0, math.radians(80.0)], time_from_start)
    time_from_start += 4
    movo_head.add_point([0.0, math.radians(-80.0)], time_from_start)
    time_from_start += 2
    movo_head.add_point([0.0, 0.0], time_from_start)
    time_from_start += 2
    movo_head.add_point([math.radians(-60.0), 0.0], time_from_start)
    # time_from_start += 4
    # movo_head.add_point([math.radians(60.0), 0.0], time_from_start)
    time_from_start += 3
    movo_head.add_point([0.0, 0.0], time_from_start)

    say(Publisher, "People say I have nice eyes. I can look up at the sky or down at the ground. "
                   "I have two laser sensors on my base, and they are great for SLAM application too. ")

    movo_head.start()
    movo_head.wait(time_from_start+2)
    print("Head Joint Trajectory Action Test Complete")

    # Ending motion
    movo_lfinger.command(0.0)
    movo_rfinger.command(0.0)
    movo_lfinger.wait(3)
    movo_rfinger.wait(3)

    say(Publisher, "If you would like to participate in my BETA phase, talk to the guys at the booth "
                   "or visit KinovaMOVO.com for more details. "
                   "Remember my name MOVO. You will be hearing a lot about me in the future. I love you guys. ")

    movo_larm.clear('left')
    movo_rarm.clear('right')
    tmp_left = rospy.wait_for_message("/movo/left_arm/joint_states", JointState)
    current_larm_pos = list(tmp_left.position)
    tmp_right = rospy.wait_for_message("/movo/right_arm/joint_states", JointState)
    current_rarm_pos = list(tmp_right.position)

    from_start_time = 0.0
    movo_larm.add_point(current_larm_pos, from_start_time)
    movo_rarm.add_point(current_rarm_pos, from_start_time)

    from_start_time += 5.0
    movo_larm.add_point_deg(larm_tuck_buff_deg, from_start_time)
    movo_rarm.add_point_deg(rarm_tuck_buff_deg, from_start_time)

    from_start_time += 5.0
    movo_larm.add_point_deg(larm_kiss_close_deg, from_start_time)
    movo_rarm.add_point_deg(rarm_kiss_close_deg, from_start_time)

    from_start_time += 5.0
    movo_larm.add_point_deg(larm_kiss_open_deg, from_start_time)
    movo_rarm.add_point_deg(rarm_kiss_open_deg, from_start_time)

    movo_larm.start()
    movo_rarm.start()
    movo_larm.wait(from_start_time+2)
    movo_rarm.wait(from_start_time+2)

    movo_lfinger.command(0.165)
    movo_rfinger.command(0.165)
    movo_lfinger.wait(3)
    movo_rfinger.wait(3)
    print("Ending Motion Complete")



    # Return to Home pose to prepare loop
    rospy.sleep(1)

    say(Publisher, "Now, please allow me to take some rest. I will be back with you in 5 minutes. Good bye. ")
    movo_lfinger.command(0.0)
    movo_rfinger.command(0.0)
    movo_lfinger.wait(3)
    movo_rfinger.wait(3)

    movo_larm.clear('left')
    movo_rarm.clear('right')
    tmp_left = rospy.wait_for_message("/movo/left_arm/joint_states", JointState)
    current_larm_pos = list(tmp_left.position)
    tmp_right = rospy.wait_for_message("/movo/right_arm/joint_states", JointState)
    current_rarm_pos = list(tmp_right.position)

    from_start_time = 0.0
    movo_larm.add_point(current_larm_pos, from_start_time)
    movo_rarm.add_point(current_rarm_pos, from_start_time)

    from_start_time += 5.0
    movo_larm.add_point_deg(larm_tuck_buff_deg, from_start_time)
    movo_rarm.add_point_deg(rarm_tuck_buff_deg, from_start_time)

    movo_larm.start()
    movo_rarm.start()
    movo_larm.wait(from_start_time+2)
    movo_rarm.wait(from_start_time+2)
    print("Return to Home position for next loop")

    process_stop_time = dt.datetime.now()
    print "Process time is : ", (process_stop_time - process_start_time).seconds, " seconds"
