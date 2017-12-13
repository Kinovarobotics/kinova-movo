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
 
 \file   movo_control_marker.py

 \brief  This module contains a collection of functions for controlling
         the Vector platform through RVIZ with interactive markers.

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""

import rospy
import tf
import sys
from utils import *
from system_defines import *
from movo_msgs.msg import *
from std_msgs.msg import UInt32
from geometry_msgs.msg import Twist
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose,Point,Quaternion,PoseStamped
import os
import threading

class MovoMarkerMenu:
    def __init__(self,server,sim):

        self.wp_menu_opt = dict({2:"Add",3:"Start",4:"Stop",5:"Reset",6:"Clear",7:"Reload",8:"Save"})
        self.mode_menu_opt = dict({10:"Standby",11:"Tractor"})
        self._server = server
        self.menu_handler = MenuHandler()
        sub_menu_handle = self.menu_handler.insert( "WayPoints" )
        self.h_wp_last = self.menu_handler.insert( "Add", parent=sub_menu_handle, callback=self._waypointCb )
        self.menu_handler.setCheckState( self.h_wp_last, MenuHandler.UNCHECKED)
        self.h_wp_last = self.menu_handler.insert( "Start", parent=sub_menu_handle, callback=self._waypointCb )
        self.menu_handler.setCheckState( self.h_wp_last, MenuHandler.UNCHECKED)
        self.h_wp_last = self.menu_handler.insert( "Stop", parent=sub_menu_handle, callback=self._waypointCb )
        self.menu_handler.setCheckState( self.h_wp_last, MenuHandler.UNCHECKED)
        self.h_wp_last = self.menu_handler.insert( "Reset", parent=sub_menu_handle, callback=self._waypointCb )
        self.menu_handler.setCheckState( self.h_wp_last, MenuHandler.UNCHECKED)
        self.h_wp_last = self.menu_handler.insert( "Clear", parent=sub_menu_handle, callback=self._waypointCb )
        self.menu_handler.setCheckState( self.h_wp_last, MenuHandler.UNCHECKED)
        self.h_wp_last = self.menu_handler.insert( "Reload", parent=sub_menu_handle, callback=self._waypointCb )
        self.menu_handler.setCheckState( self.h_wp_last, MenuHandler.UNCHECKED)
        self.h_wp_last = self.menu_handler.insert( "Save", parent=sub_menu_handle, callback=self._waypointCb )
        self.menu_handler.setCheckState( self.h_wp_last, MenuHandler.UNCHECKED)
        
        
        sub_menu_handle = self.menu_handler.insert( "Mode" )
        self.h_mode_last = self.menu_handler.insert( "Standby", parent=sub_menu_handle, callback=self._modeCb )
        self.menu_handler.setCheckState( self.h_mode_last, MenuHandler.UNCHECKED)
        self.h_mode_last = self.menu_handler.insert( "Tractor", parent=sub_menu_handle, callback=self._modeCb )
        self.menu_handler.setCheckState( self.h_mode_last, MenuHandler.UNCHECKED)
        
    
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.pose.position.z = 2.0
        int_marker.scale = 1
        int_marker.name = "movo_menu_marker"
        marker_box = Marker()
        marker_box.type = Marker.CUBE
        marker_box.scale.x = int_marker.scale * 0.45
        marker_box.scale.y = int_marker.scale * 0.45
        marker_box.scale.z = int_marker.scale * 0.45
        marker_box.color.r = 0.5
        marker_box.color.g = 0.5
        marker_box.color.b = 0.5
        marker_box.color.a = 1.0
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.always_visible = False
        control.markers.append(marker_box)
        int_marker.controls.append(control)
        self._server.insert( int_marker, self._clicked )
             
        self.menu_handler.apply(self._server, "movo_menu_marker")
        
        self._server.applyChanges()
        
        self._msg_pub = rospy.Publisher('/movo/waypoint_cmd',UInt32,queue_size=10)
        self._cfg_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=10)            
        
    def _waypointCb(self,feedback):
        handle = feedback.menu_entry_id
        self.menu_handler.setCheckState( handle, MenuHandler.CHECKED )
        
        if ("Add" == self.wp_menu_opt[handle]):
            msg = 1<<0
        elif ("Start" == self.wp_menu_opt[handle]):
            msg = 1<<1
        elif ("Stop" == self.wp_menu_opt[handle]):
            msg = 1<<2
        elif ("Reset" == self.wp_menu_opt[handle]):
            msg = 1<<3
        elif ("Clear" == self.wp_menu_opt[handle]):
            msg = 1<<4
        elif ("Save" == self.wp_menu_opt[handle]):
            msg = 1<<5
        elif ("Reload" == self.wp_menu_opt[handle]):
            msg = 1<<6                  
        self._msg_pub.publish(msg)
        for key,value in self.wp_menu_opt.iteritems():
            if (key != handle):
                self.menu_handler.setCheckState( key, MenuHandler.UNCHECKED )

        self.menu_handler.reApply( self._server )
        self._server.applyChanges()

    def _modeCb(self,feedback):
        handle = feedback.menu_entry_id
        self.menu_handler.setCheckState( handle, MenuHandler.CHECKED )
        msg = ConfigCmd()
        msg.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
        if ("Standby" == self.mode_menu_opt[handle]):
            msg.gp_param = STANDBY_REQUEST
        elif ("Tractor" == self.mode_menu_opt[handle]):
            msg.gp_param = TRACTOR_REQUEST
        
        self._cfg_pub.publish(msg)
        
        for key,value in self.mode_menu_opt.iteritems():
            if (key != handle):
                self.menu_handler.setCheckState( key, MenuHandler.UNCHECKED )

        self.menu_handler.reApply( self._server )
        self._server.applyChanges()
    def _clicked(self,feedback):
        pass
        
        
class MovoMarkerControl:
    def __init__(self,sim):
        self.lock=threading.RLock()
    
        """
        Subscribe to the configuration message
        """
        if (False == sim):
            self.config_updated = False
            rospy.Subscriber("/movo/feedback/active_configuration", Configuration, self._update_configuration_limits)
            start_time = rospy.get_time()            
            while ((rospy.get_time() - start_time) < 10.0) and (False == self.config_updated):
                rospy.sleep(0.05)
            
            if (False == self.config_updated):
                rospy.logerr("Timed out waiting for Movo feedback topics make sure the driver is running")
                sys.exit(0)
                return
        else:
            self.x_vel_limit_mps = 0.5
            self.y_vel_limit_mps = 0.5
            self.yaw_rate_limit_rps = 0.5
            self.accel_lim = 1.0
            self.yaw_accel_lim = 1.0
        
                
        # create an interactive marker server on the topic namespace simple_marker
        self._server = InteractiveMarkerServer("movo_marker_ctrl")
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.last_marker_update = rospy.get_time()
        self.last_feedback = None
        
        self.linear_scale = rospy.get_param('~linear_scale', 1.0);
        self.angular_scale = rospy.get_param('~angular_scale', 2.2);
        
        self.motion_cmd = Twist()
        self.motion_pub = rospy.Publisher('/movo/int_marker/cmd_vel', Twist, queue_size=10)

        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "movo_twist_ctrl"
        int_marker.description = "MoVo Control Marker"
        
        control = InteractiveMarkerControl()
        control.orientation_mode = InteractiveMarkerControl.FIXED;
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS;
        int_marker.controls.append( control )
        
        control = InteractiveMarkerControl()
        control.orientation_mode = InteractiveMarkerControl.FIXED;
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS;
        int_marker.controls.append( control )

        control = InteractiveMarkerControl()
        control.orientation_mode = InteractiveMarkerControl.FIXED;
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS;
        int_marker.controls.append( control ) 

        # add the interactive marker to our collection &
        # tell the server to call processFeedback() when feedback arrives for it
        self._server.insert(int_marker, self.processFeedback)

        # 'commit' changes and send to all clients
        self._server.applyChanges()
        
        MovoMarkerMenu(self._server,sim)
        
        rospy.Timer(rospy.Duration(0.5),self.timeout_check)
        
    def timeout_check(self,event):
    
        now_time = rospy.get_time()
        dt = now_time - self.last_marker_update
        
        if (dt > 0.4) and (None != self.last_feedback) and (True == self.stop_on_timeout):
            self.motion_cmd.linear.x = 0.0
            self.motion_cmd.linear.y = 0.0
            self.motion_cmd.angular.z = 0.0
            self.motion_pub.publish(self.motion_cmd)
            self._server.setPose(self.last_feedback.marker_name, Pose())
            self._server.applyChanges()
            self.stop_on_timeout = False       
        
    def _update_configuration_limits(self,config):
        self.x_vel_limit_mps = config.teleop_x_vel_limit_mps
        self.y_vel_limit_mps = config.teleop_y_vel_limit_mps
        self.yaw_rate_limit_rps = config.teleop_yaw_rate_limit_rps
        self.accel_lim = config.teleop_accel_limit_mps2
        self.yaw_accel_lim = config.teleop_yaw_accel_limit_rps2
        self.config_updated = True

    def processFeedback(self,feedback):
        with self.lock:
        
            p = feedback.pose.position
            o = feedback.pose.orientation
            
            now_time = rospy.get_time()
            dt = now_time - self.last_marker_update
            self.last_feedback = feedback
            
                    
            if (dt >= 0.01):
                self.last_marker_update = now_time
                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
                vx = p.x * self.linear_scale
                vx = limit_f(vx,self.x_vel_limit_mps)
                vy = p.y * self.linear_scale
                vy = limit_f(vy,self.y_vel_limit_mps)
                wz = yaw * self.angular_scale          
                wz = limit_f(wz,self.yaw_rate_limit_rps)

                self.motion_cmd.linear.x = slew_limit(vx,
                                                      self.motion_cmd.linear.x,
                                                      self.accel_lim, dt)
                self.motion_cmd.linear.y = slew_limit(vy,
                                                      self.motion_cmd.linear.y,
                                                      self.accel_lim, dt)
                
                self.motion_cmd.angular.z = slew_limit(wz,
                                                       self.motion_cmd.angular.z,
                                                       self.yaw_accel_lim, dt)
            

                self.motion_pub.publish(self.motion_cmd)
                self.stop_on_timeout = True
                
            
            self._server.setPose(feedback.marker_name, Pose())
            self._server.applyChanges()
        


