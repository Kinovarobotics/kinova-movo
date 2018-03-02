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

 \file   system_test.py

 \brief

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import rospy
import random
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from jaco_jtas_test import JacoJTASTest
from head_jtas_test import HeadJTASTest  
from torso_jtas_test import TorsoJTASTest 
from gripper_action_test import GripperActionTest

class MovoVoiceTest(object):

    def __init__(self):   
        self.voice_cmd = String()
        self.pub = rospy.Publisher("/movo/voice/text", String, queue_size=1, latch=True)
    def say(self,data):
        self.voice_cmd.data = data
        self.pub.publish(self.voice_cmd)
        """
        rospy.sleep(3.0)
        self.pub.unregister()
        self.pub = rospy.Publisher("/movo/voice/text", String, queue_size=1, latch=True)    
        """        



if __name__ == "__main__":
    rospy.init_node('system_test')
    dof = rospy.get_param('~jaco_dof')

    voice = MovoVoiceTest() 
    voice.say("Hello,       I am movo.          I can be your best friend!")
    
    h_test  = HeadJTASTest()
    l_test = JacoJTASTest(arm='left', dof=dof)
    r_test = JacoJTASTest(arm='right', dof=dof)
    rg_test = GripperActionTest("right")
    lg_test = GripperActionTest("left")
    t_test = TorsoJTASTest()
    
    """
    Test Head
    """
    tmp = rospy.wait_for_message("/movo/head/joint_states", JointState)
    current_angles= tmp.position
    h_test.add_point(list(current_angles), 0.0)
    
    total_time = 0.0
    points = [list(current_angles), 0.0]
    for i in range(0,5):
        
        pos = [random.uniform(-1.57,1.57),random.uniform(-1.57,1.57)]
        vel = random.uniform(0.2,0.5)
        
        dt = 0.0
        for i in range(2):
            tmp = abs(pos[i])/vel
            if (tmp > dt):
                dt = tmp
        total_time+=dt
   
        h_test.add_point(pos,total_time)
        
    total_time+=5.0
    h_test.add_point([0.0,0.0],total_time)
        
    h_test.start()
    h_test.wait(total_time+3.0)
    print("Head Joint Trajectory Action Test Complete")
    
    """
    Gripper test
    """
    lg_test.command(0.0)
    rg_test.command(0.0)
    lg_test.wait()
    rg_test.wait()
    
    lg_test.command(0.085)
    rg_test.command(0.085)
    lg_test.wait()
    rg_test.wait()
    
    lg_test.command(0.165)
    rg_test.command(0.165)
    lg_test.wait()
    rg_test.wait()
    
    lg_test.command(0.0)
    rg_test.command(0.0)
    lg_test.wait()
    rg_test.wait()
    
    print("Gripper Action Test Example Complete")
    
    """
    Left Arm Test
    """
    tmp = rospy.wait_for_message("/movo/left_arm/joint_states", JointState)
    current_angles= list(tmp.position)
    l_test.add_point(current_angles, 0.0)
    
    if '6dof' == dof:
        p1 = [0.0] * 6
    if '7dof' == dof:
        p1 = [0.0] * 7

    l_test.add_point(p1,10.0)
    l_test.add_point(current_angles,20.0)
    l_test.start()

    l_test.wait(23.0)
    print("Right Arm Joint Trajectory Action Test Complete")
    
    
    
    """
    Left Arm Test
    """
    tmp = rospy.wait_for_message("/movo/right_arm/joint_states", JointState)
    current_angles= list(tmp.position)
    r_test.add_point(current_angles, 0.0)

    if '6dof' == dof:
        p1 = [0.0] * 6
    if '7dof' == dof:
        p1 = [0.0] * 7

    r_test.add_point(p1,10.0)
    r_test.add_point(current_angles,20.0)
    r_test.start()

    r_test.wait(23.0)
    print("Right Arm Joint Trajectory Action Test Complete")
      
    """
    Torso Test
    """
    tmp = rospy.wait_for_message("/movo/linear_actuator/joint_states", JointState)
    current_angles= list(tmp.position)
    t_test.add_point(current_angles, 0.0)
    
    total_time = 0.0
    for i in range(0,5):
        pos = random.uniform(0.0,0.4)
        vel = random.uniform(0.02,0.06)
        dt = abs(pos)/vel
        total_time+=dt
        t_test.add_point([pos],total_time)

    t_test.start()
    t_test.wait(total_time+3.0)
    
    print("Torso Joint Trajectory Action Test Complete")
    
    print
    print("----------------------------------------------------")
    print("Exiting - All Joint Trajectory Action Test Complete")
    print("----------------------------------------------------")
    
    
    
    
    
    
    
    
    
    
    
    
    
