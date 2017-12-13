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
 
 \file   movo_linear_actuator.py

 \brief  runs the driver for the linear actuator

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from system_defines import *
from utils import *
from movo_msgs.msg import LinearActuatorCmd
from io_eth import IoEthThread
import multiprocessing
import re
import os
import rospy

class LinearActuator(object):
    def __init__(self,movo_ip='10.66.171.5'):        
        self.init_success = False
        
        """
        Create the thread to run MOVO Linear actuator command interface
        """
        self.tx_queue_ = multiprocessing.Queue()
        self.rx_queue_ = multiprocessing.Queue()
        
        self.comm = IoEthThread((movo_ip,6236),
                                self.tx_queue_,
                                self.rx_queue_,
                                max_packet_size=1248)
                                    
        
        if (False == self.comm.link_up):
            rospy.logerr("Could not open socket for MOVO linear actuator...exiting")
            self.Shutdown()
            return
        
        """
        Initialize the publishers and subscribers for the node
        """
        self.cmd_data = LinearActuatorCmd()
        self.s = rospy.Subscriber("/movo/linear_actuator_cmd", LinearActuatorCmd, self._add_motion_command_to_queue)
        self.init_success = True
    
    def Shutdown(self):
        rospy.loginfo("Shutting down the linear actuator command driver...")
        self.s.unregister()
        self.comm.Close()
        self.tx_queue_.close()
        self.rx_queue_.close()
        
    def UpdateVelLimit(self,new_limit):
        cmds = [LINEAR_ACTUATOR_VELOCITY_LIMIT_CMD_ID,[convert_float_to_u32(new_limit)]]
        self._add_command_to_queue(cmds) 
    
    def _add_command_to_queue(self,command):
        
        if (rospy.is_shutdown()):
            return
        """
        Create a byte array with the CRC from the command
        """
        cmd_bytes = generate_cmd_bytes(command)
        
        """
        Send it
        """
        self.tx_queue_.put(cmd_bytes)
        
    def _add_motion_command_to_queue(self,command):
        if (rospy.is_shutdown()):
            return        
        """
        Add the command to the queue, platform does command limiting and mapping
        """
        cmds = [LINEAR_ACTUATOR_POSITION_CMD_ID,[convert_float_to_u32(command.desired_position_m)]]
        self._add_command_to_queue(cmds)
