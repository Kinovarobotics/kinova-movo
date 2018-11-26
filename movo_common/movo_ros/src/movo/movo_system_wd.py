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
 
 \file   movo_system_wd.py

 \brief  This is the movo system watchdod which monitors signals
         from the embedded power system to safely shutdown the PC
         upon embedded powerdown

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import socket
import sys
import rospy
import os
from std_msgs.msg import Bool
from utils import m32
import socket
import re

class MovoWatchdog:
    def __init__(self,pc_name):
        """
        Initialize the UDP connection
        """
        self._remote_shutdown_msg = Bool()
        self._remote_shutdown_msg.data = True
        if (pc_name == 'movo1'):
            self.remote_sub = rospy.Subscriber('movo1/shutdown_pc',Bool,self._shutdown_cb)
            self.remote_pub = rospy.Publisher('movo2/shutdown_pc',Bool,queue_size=1,latch=True)
        else:
            self.remote_sub = rospy.Subscriber('movo2/shutdown_pc',Bool,self._shutdown_cb)
            self.remote_pub = rospy.Publisher('movo1/shutdown_pc',Bool,queue_size=1,latch=True)
            
        self._continue = True     
        self.conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.conn.setblocking(False)
        self.conn.bind(('',6234))
        
    def _shutdown_cb(self,msg):
        rospy.logerr("Platform signaled shutdown, need to shutdown the onboard PC")
        self.Close()
        os.system("sudo shutdown now -h")
        sys.exit(0)
            
    def Receive(self):
        """
        Try receiving the data up to a maximum size. If it fails
        empty the data
        """
        try:
            data = self.conn.recv(4)
        except:
            data = []            
            
        if (len(data) == 4):
        
            rx_dat = [ord(i) for i in data]
            shutdwn_cmd = m32(rx_dat)
            if (0x8756BAEB == shutdwn_cmd):
                self.remote_pub.publish(self._remote_shutdown_msg)
                rospy.sleep(1.0)
                self._shutdown_cb(True)
    def Close(self):
        self.conn.close()       
        
    
    
    
    
    
