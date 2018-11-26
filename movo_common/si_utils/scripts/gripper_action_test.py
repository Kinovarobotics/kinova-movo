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

 \file   gripper_action_test.py

 \brief  ROS Driver for interfacing with the Kinova Jaco integrated 
         with the Stanley Innovation Vector platform

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import sys
from copy import copy
import rospy
import actionlib
import math
import random

from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)

from sensor_msgs.msg import JointState


class GripperActionTest(object):
    def __init__(self,prefix="right"):
        
        self._prefix = prefix
        self._client = actionlib.SimpleActionClient(
            '/movo/%s_gripper_controller/gripper_cmd'%self._prefix,
            GripperCommandAction,
        )
        self._goal = GripperCommandGoal()
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Gripper Command"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def command(self, position, block=False, timeout=15.0):
        self._goal.command.position = position
        self._goal.command.max_effort = -1.0
        self._client.send_goal(self._goal)
        if block:
            self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = GripperCommandGoal()


def main():
    rospy.init_node('gripper_action_test')

    rg_test = GripperActionTest("right")
    lg_test = GripperActionTest("left")
    
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
    
if __name__ == "__main__":
    main()

        
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
