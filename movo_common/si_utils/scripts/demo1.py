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
from sensor_msgs.msg import JointState
from head_jtas_test import HeadJTASTest
from voice_test import MovoVoiceTest
from jaco_jtas_test import JacoJTASTest
from gripper_action_test import GripperActionTest
from torso_jtas_test import TorsoJTASTest
from base_motion_test import BaseMotionTest

import datetime as dt

if __name__ == "__main__":

    process_start_time = dt.datetime.now()
    rospy.init_node("demo1")

    movo_head = HeadJTASTest()
    movo_voice = MovoVoiceTest()
    movo_base = BaseMotionTest()
    movo_larm = JacoJTASTest('left')
    movo_rarm = JacoJTASTest('right')
    movo_lfinger = GripperActionTest('left')
    movo_rfinger = GripperActionTest('right')
    movo_torsor = TorsoJTASTest()

    rospy.sleep(2)

    # arm position definition
    larm_home_deg = [85, 13, 122.5, 120, -83, -75]
    rarm_home_deg = [-1.0 * x for x in larm_home_deg]

    larm_greet_deg = [125, -30, 45, 120, 0, -170]
    rarm_greet_deg = [-1.0 * x for x in larm_greet_deg]

    larm_traj_side_deg = [179, -90, 0, 0, 0, -179]
    rarm_traj_side_deg = [-1.0 * x for x in larm_traj_side_deg]

    larm_traj_down_deg = [270, -45, -45, 90, 0, -179]
    rarm_traj_down_deg = [-1.0 * x for x in larm_traj_down_deg]

    larm_kiss_close_deg = [34, 45, 130, 90, -45, 0]
    rarm_kiss_close_deg = [-1.0 * x for x in larm_kiss_close_deg]

    larm_kiss_open_deg = [45, 45, 20, 125, -15, -15]
    rarm_kiss_open_deg = [-1.0 * x for x in larm_kiss_open_deg]

    larm_tuck_deg = [90, 90, 160, 120, 0, 0]
    rarm_tuck_deg = [-1.0 * x for x in larm_tuck_deg]

    larm_tuck_buff_deg = [80, 80, 150, 140, -20, 0]
    rarm_tuck_buff_deg = [-1.0 * x for x in larm_tuck_buff_deg]

    """
    1. Greeting words
    """

    movo_voice.say("Hello there. My name is MOVO. "
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
    movo_voice.say("I can move all over the place. Forward... and backward... and to the side. "
                   "Do I look like Michael Jackson? ")
    movo_base.move_forward(0.2)
    movo_base.move_backward(0.165)
    rospy.sleep(1)

    movo_base.move_left(0.3, 0.15)
    rospy.sleep(1)
    movo_base.move_right(0.29, 0.15)
    rospy.sleep(1)

    movo_voice.say("I can even do a pirouette. ")
    movo_base.rotate_clock(30)
    rospy.sleep(1)
    movo_base.rotate_anticlock(30.5)
    print "Movo Completing Base Motion"

    # Torso motion
    movo_voice.say("There are certain advantages to being a robot. We do not have to worry about being too tall or too short. "
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
    movo_voice.say("Thanks to six degrees of freedom, I can move my arms in many ways. "
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
    movo_voice.say("I have three fingers--they are designed to help me perform thousands of tasks")
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

    movo_voice.say("People say I have nice eyes. I can look up at the sky or down at the ground. "
                   "I have two laser sensors on my base, and they are great for SLAM application too. ")

    movo_head.start()
    movo_head.wait(time_from_start+2)
    print("Head Joint Trajectory Action Test Complete")

    # Ending motion
    movo_lfinger.command(0.0)
    movo_rfinger.command(0.0)
    movo_lfinger.wait(3)
    movo_rfinger.wait(3)

    movo_voice.say("If you would like to participate in my BETA phase, talk to the guys at the booth "
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

    movo_voice.say("Now, please allow me to take some rest. I will be back with you in 5 minutes. Good bye. ")
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
