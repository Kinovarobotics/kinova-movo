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
 
 \file   rmp_comm.py

 \brief  runs the driver

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from movo_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from movo.system_defines import *
import rospy
import tf


class Helpers(object):
    def __init__(self):
        self._battery_low = False
        self._dyn_rsp_active = False
        self._operational_state = 0
        self.tfl = tf.TransformListener()
        rospy.sleep(3.0)        
        
        self.cmd_config_cmd_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/movo/manual_override/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/movo/feedback/battery", Battery, self._handle_low_battery)
        rospy.Subscriber("/movo/feedback/status", Status, self._handle_status)


            
    def BatteryIsLow(self):
        return self._battery_low

    def GetRobotOpState(self):
        return self._operational_state
        
    def SetRobotMode(self,mode):        
        """
        define the commands for the function
        """
        config_cmd = ConfigCmd()
        
        """
        Send the audio command
        """
        attempts = 5
        success = False
        start_time = rospy.get_time()
        while ((rospy.get_time() - start_time) < 10.0) and not success:
            config_cmd.header.stamp = rospy.get_rostime()
            config_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
            config_cmd.gp_param = mode
            self.cmd_config_cmd_pub.publish(config_cmd)
            if MOVO_MODES_DICT[mode]  == self._operational_state:
                success = True
            attempts+=1

        if not success:
            rospy.logerr("Could not set operational Mode")
            rospy.loginfo("The platform did not respond, ")
            return False
        return True
    
    def StopRobotMotion(self):
        self.t1 = rospy.Timer(rospy.Duration(0.01),self._send_zero_command)

    def ResumeRobotMotion(self):
        try:
            self.t1.shutdown()
        except:
            pass

    def GetCurrentRobotPose(self,frame="map"):
        self.tfl.waitForTransform(frame, "base_link", rospy.Time(), rospy.Duration(1.0))
        (trans,rot) = self.tfl.lookupTransform(frame, "base_link", rospy.Time(0))
        
        """
        Remove all the rotation components except yaw
        """
        euler = tf.transformations.euler_from_quaternion(rot)
        rot = tf.transformations.quaternion_from_euler(0,0,euler[2])    
    
        current_pose = PoseWithCovarianceStamped()
        current_pose.header.stamp = rospy.get_rostime()
        current_pose.header.frame_id = frame
        current_pose.pose.pose = Pose(Point(trans[0], trans[1], 0.0), Quaternion(rot[0],rot[1],rot[2],rot[3]))
        
        return current_pose
    
    def _send_zero_command(self):
        self.cmd_vel_pub.publish(Twist())
        
    def _handle_low_battery(self, battery_msg ):
        if (battery_msg.battery_soc < 5.0):
            self._battery_low = True

    def _handle_status(self,stat):
        if stat.dynamic_response != 0: 
            self._dyn_rsp_active = True
        
        self._operational_state = stat.operational_state
        
    


        
