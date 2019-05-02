"""--------------------------------------------------------------------

Copyright (c) 2019, Kinova Robotics inc.


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
 
 \file   movo_teleop_full_system_xbox.py

 \brief  This module contains a class for teleoperating all the movo
         platform DOF with a joystick controller; only works with Xbox controller

 \Platform: Ubuntu 16.04 LTS / ROS Kinetic

Edited 7/25/2016: Vivian Chu, vchu@gatech - included support for simulation
Edited 11/07/2016: David Kent, dekent@gatech - integrated arm commands with wpi_jaco

--------------------------------------------------------------------"""
from utils import *
from system_defines import *
from movo_msgs.msg import *
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,Float64,Float32, String
from trajectory_msgs.msg import JointTrajectoryPoint  
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState
from movo_msgs.msg import JacoCartesianVelocityCmd,PanTiltCmd
from topic_tools.srv import MuxSelect, MuxSelectRequest
import rospy
import sys
import math
import actionlib


class MovoTeleopFullSystem(object):
    def __init__(self):
        self.is_sim = rospy.get_param('~sim',True)
        self.lincmd = LinearActuatorCmd()
        if (False == self.is_sim):
        
            """
            Subscribe to the configuration message
            """
            self.config_updated = False
            rospy.Subscriber("/movo/feedback/active_configuration", Configuration, self._update_configuration_limits)
            
            start_time = rospy.get_time()            
            while ((rospy.get_time() - start_time) < 10.0) and (False == self.config_updated):
                rospy.sleep(0.05)
            
            if (False == self.config_updated):
                rospy.logerr("Timed out waiting for Movo feedback topics make sure the driver is running")
                sys.exit(0)
                return

            """
            Initialize the linear actuator position if this is the real system
            """
            movo_dynamics = rospy.wait_for_message("/movo/feedback/dynamics", Dynamics)
            self.lincmd.desired_position_m = movo_dynamics.linear_actuator_position_m
            
        else:
            self.x_vel_limit_mps = rospy.get_param('~sim_teleop_x_vel_limit_mps',0.5)
            self.y_vel_limit_mps = rospy.get_param('~sim_teleop_y_vel_limit_mps',0.5)
            self.yaw_rate_limit_rps = rospy.get_param('~sim_teleop_yaw_rate_limit_rps',0.5)
            self.accel_lim = rospy.get_param('~sim_teleop_accel_lim',0.5)
            self.yaw_accel_lim = rospy.get_param('~sim_teleop_yaw_accel_lim',1.0)
            self.arm_vel_lim = rospy.get_param('~sim_teleop_arm_vel_limit',0.1)
            self.pt_vel_lim = rospy.get_param('~sim_teleop_pan_tilt_vel_limit',0.524)
            self.lin_act_vel_lim = rospy.get_param('~sim_teleop_linear_actuator_vel_limit',0.05)
    
        """
        Set the mapping for the various commands. Take the wired microsoft xbox 360
        """        
        self.ctrl_map  = dict({'momentary': {'MAP_BASE_A'     : {'is_button':True,'index':1,'set_val':1},
                                             'MAP_rARM_B'    : {'is_button':True,'index':2,'set_val':1},
                                             'MAP_lARM_X'      : {'is_button':True,'index':3,'set_val':1},
                                             'MAP_HEAD_Y'      : {'is_button':True,'index':4,'set_val':1},
                                             'MAP_lWRIST_LB'       : {'is_button':True,'index':5,'set_val':1},
                                             'MAP_rWRIST_RB'       : {'is_button':True,'index':6,'set_val':1},
                                             'MAP_TORS_back_'        : {'is_button':True,'index':7,'set_val':1},
                                             'MAP_HOME_start'    : {'is_button':True,'index':8,'set_val':1},
                                             'MAP_eSTOP_power' : {'is_button':True,'index':9,'set_val':1},
                                             'MAP_TRACT_lstick'     : {'is_button':True,'index':10,'set_val':1},
                                             'MAP_stby_rstick': {'is_button':True,'index':11,'set_val':1}},
                               'axis'     : {'MAP_TWIST_LR_stickleft'   : {'index' :1, 'invert_axis':False},
                                             'MAP_TWIST_UD_stickleft'      : {'index' :2, 'invert_axis':False},
                                             'MAP_TRIGGER_LT'        : {'index' :3, 'invert_axis':False},
                                             'MAP_TWIST_LR_stickright'   : {'index' :4, 'invert_axis':False},
                                             'MAP_TWIST_UD_stickright'   : {'index' :5, 'invert_axis':False},
                                             'MAP_TRIGGER_RT'   : {'index' :6, 'invert_axis':False},
                                             'MAP_CROSS_LR'   : {'index' :7, 'invert_axis':False},
                                             'MAP_CROSS_UD'   : {'index' :8, 'invert_axis':False}}})
        
        """
        Initialize the debounce logic states
        """
	
        self.db_cnt = dict()
        self.axis_value = dict()
        self.button_state = dict()
        
        for key, value in self.ctrl_map.iteritems():
            if key == 'momentary':
                for key, value2 in value.iteritems():
                    self.db_cnt[key]=0
                    self.button_state[key]=False
            else:
                self.db_cnt[key]=0
                self.axis_value[key]=0.0            
                 
        self.send_cmd_none = False
        self.no_motion_commands = True
        self.last_motion_command_time = 0.0
        self.last_joy = rospy.get_time()
        self._last_gripper_val = 0.0
        self.run_arm_ctl_right = False
        self.run_arm_ctl_left = False
        self.run_pan_tilt_ctl = False
        self._init_pan_tilt = True
        self._last_angles = [0.0,0.0]
        self._pt_pos_cmd = [0.0,0.0]
            
        self.cfg_cmd = ConfigCmd()
        self.cfg_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=10)
        
        self.motion_cmd = Twist()
        self.limited_cmd = Twist()
        self.motion_pub = rospy.Publisher('/movo/teleop/cmd_vel', Twist, queue_size=10)
        self.override_pub = rospy.Publisher("/movo/manual_override/cmd_vel",Twist, queue_size=10)
        self.linpub = rospy.Publisher("/movo/linear_actuator_cmd",LinearActuatorCmd,queue_size=1)
        
        self.arm_pub = [0]*2
        self.gripper_pub = [0]*2
        self.kgripper_pub = [0]*2
        
        self.home_arm_pub = rospy.Publisher('/movo/home_arms', Bool, queue_size=10)
        self.last_home_req = False
        self.home_req_timeout = 0.0
        self.homing_sent = False 
        
        self.arm_pub[0] = rospy.Publisher('/movo/right_arm/cartesian_vel_cmd', JacoCartesianVelocityCmd, queue_size=10)
        self.gripper_pub[0] = rospy.Publisher('/movo/right_gripper/cmd', GripperCmd, queue_size=10)
        self.kgripper_pub[0] = rospy.Publisher('/movo/right_gripper/vel_cmd', Float32, queue_size=10)

        self.arm_pub[1] = rospy.Publisher('/movo/left_arm/cartesian_vel_cmd', JacoCartesianVelocityCmd, queue_size=10)
        self.gripper_pub[1] = rospy.Publisher('/movo/left_gripper/cmd', GripperCmd, queue_size=10)
        self.kgripper_pub[1] = rospy.Publisher('/movo/left_gripper/vel_cmd', Float32, queue_size=10)
        
        self.p_pub = rospy.Publisher("/movo/head/teleop/cmd", PanTiltCmd, queue_size=100)
        self.pt_cmds = PanTiltCmd()

        self.head_cmd_srv = rospy.ServiceProxy("/head_cmd_mux/select", MuxSelect)
        self.head_cmd_source = ['teleop', 'face_tracking']
        self.head_cmd_current_source_index = -1

        self.teleop_control_mode_pub = rospy.Publisher("/movo/teleop/control_mode", String, queue_size = 10)
        self.teleop_control_mode_msg = String()

        rospy.Subscriber('/joy', Joy, self._movo_teleop)
 
    def _update_configuration_limits(self,config):
        
        self.x_vel_limit_mps = config.teleop_x_vel_limit_mps
        self.y_vel_limit_mps = config.teleop_y_vel_limit_mps
        self.yaw_rate_limit_rps = config.teleop_yaw_rate_limit_rps
        self.accel_lim = config.teleop_accel_limit_mps2
        self.yaw_accel_lim = config.teleop_yaw_accel_limit_rps2
        self.arm_vel_lim = config.teleop_arm_vel_limit
        self.pt_vel_lim = config.teleop_pan_tilt_vel_limit
        self.lin_act_vel_lim = config.teleop_linear_actuator_vel_limit
        self.config_updated = True
        
    def _parse_joy_input(self,joyMessage):
        
    
        raw_button_states = dict()
        self.button_state = dict()
        
        for key, value in self.ctrl_map.iteritems():
            if key == 'momentary':
                for key2, value2 in value.iteritems():
                    raw_button_states[key2]=True
                    self.button_state[key2]=False
            else:
                for key2, value2 in value.iteritems():  
                    self.axis_value[key2] = 0.0            
         
        
        for key, value in self.ctrl_map.iteritems():
            if key == 'momentary':
                for key2, item in value.iteritems():
                    if item['is_button']:
                        if item['set_val'] == joyMessage.buttons[item['index']-1]:
                            raw_button_states[key2] &= True
                        else:
                            raw_button_states[key2] = False
                    else:
                        temp = joyMessage.axes[item['index']-1]
                        if (item['invert_axis']):
                            temp *= -1.0
                        if (temp >= item['set_thresh']):
                            raw_button_states[key2] &= True
                        else:
                            raw_button_states[key2] = False
                     
            
                    if (True == raw_button_states[key2]):
                        self.db_cnt[key2]+=1
                        if (self.db_cnt[key2] > 10):
                            self.db_cnt[key2] = 10
                            self.button_state[key2] = True
                    else:
                        self.button_state[key2] = False
                        self.db_cnt[key2] = 0
            if key == 'axis':
                for key2, item in value.iteritems():
                    temp = joyMessage.axes[item['index']-1]
                    if (item['invert_axis']):
                        temp *= -1.0
                    self.axis_value[key2] = temp
                    
                    

    def _movo_teleop(self, joyMessage):
        self._parse_joy_input(joyMessage)
        dt = rospy.get_time() - self.last_joy
        self.last_joy = rospy.get_time()
        
        if self.button_state['MAP_BASE_A']:
            self.teleop_control_mode_msg.data = "base_ctl"
            self.run_arm_ctl_right = False
            self.run_arm_ctl_left = False
            self.run_pan_tilt_ctl = False
            self._init_pan_tilt = False
        elif self.button_state['MAP_rARM_B']:
            self.teleop_control_mode_msg.data = "arm_ctl_right"
            self.run_arm_ctl_right = True
            self.run_arm_ctl_left = False
            self.run_pan_tilt_ctl = False
            self._init_pan_tilt = False
        elif self.button_state['MAP_lARM_X']:
            self.teleop_control_mode_msg.data = "arm_ctl_left"
            self.run_arm_ctl_right = False
            self.run_arm_ctl_left = True
            self.run_pan_tilt_ctl = False
            self._init_pan_tilt = False
        elif self.button_state['MAP_HEAD_Y']:
            self.teleop_control_mode_msg.data = "pan_tilt_ctl"
            # only mux topic when run_pan_tilt_ctl is changed from False to true
            if self.run_pan_tilt_ctl == False:
                self.head_cmd_current_source_index = (self.head_cmd_current_source_index + 1) % len(self.head_cmd_source)
                rospy.wait_for_service('/head_cmd_mux/select')
                switch_request = MuxSelectRequest()
                switch_request.topic = '/movo/head/' + self.head_cmd_source[self.head_cmd_current_source_index] + '/cmd'
                self.head_cmd_srv.call(switch_request)

            self.run_arm_ctl = False
            self.run_arm_ctl_right = False
            self.run_arm_ctl_left = False
            self.run_pan_tilt_ctl = True
            self._init_pan_tilt = True

        if self.button_state['MAP_eSTOP_power']:
            self.teleop_control_mode_msg.data = "estop"
            self.run_arm_ctl = False
            self.run_pan_tilt_ctl = False
            self._init_pan_tilt = False
            arm_cmd = JacoCartesianVelocityCmd()
            arm_cmd.header.stamp=rospy.get_rostime()
            arm_cmd.header.frame_id=''
            self.arm_pub[0].publish(arm_cmd)
            self.arm_pub[1].publish(arm_cmd)

        if self.button_state['MAP_HOME_start']:
            self.teleop_control_mode_msg.data = "home_arms"
            if not (self.homing_sent):
                tmp = Bool()
                tmp.data = True
                self.home_arm_pub.publish(tmp)
                self.homing_sent = True
                self.homing_start_time = rospy.get_time()
                
        if (self.homing_sent):
            if ((rospy.get_time()-self.homing_start_time) > 10.0) and not self.button_state['MAP_HOME_start']:
                self.homing_sent = False
                
        if self.button_state['MAP_eSTOP_power']:
            self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
            self.cfg_cmd.gp_param = DISABLE_REQUEST
            self.cfg_cmd.header.stamp = rospy.get_rostime()
            self.cfg_pub.publish(self.cfg_cmd)
            self.cfg_cmd.header.seq 


        """
        publish current control mode to inform other nodes
        """
        self.teleop_control_mode_pub.publish(self.teleop_control_mode_msg)

    
        if self.run_arm_ctl_right or self.run_arm_ctl_left:

            arm_cmd = JacoCartesianVelocityCmd()
            arm_cmd.header.stamp=rospy.get_rostime()
            arm_cmd.header.frame_id=''
            gripper_cmd = GripperCmd()
            kg_cmd = Float32()
            
            if self.run_arm_ctl_right:
                arm_idx = 0
            else:
                arm_idx = 1
  
            if self.axis_value['MAP_TRIGGER_RT']==-1.0:
                if 0==arm_idx:
                    arm_cmd.x = self.axis_value['MAP_TWIST_LR_stickleft'] * -self.arm_vel_lim
                else:
                    arm_cmd.x = self.axis_value['MAP_TWIST_LR_stickleft'] * -self.arm_vel_lim
                arm_cmd.z = self.axis_value['MAP_TWIST_UD_stickleft'] * self.arm_vel_lim
                
                if not self.button_state['MAP_TORS_back_']:
                    arm_cmd.y = self.axis_value['MAP_TWIST_UD_stickright'] * -self.arm_vel_lim
                else:
                    dt = rospy.get_time() - self.last_arm_update
                    self.lincmd.desired_position_m += (self.axis_value['MAP_TWIST_UD_stickright'] * self.lin_act_vel_lim) * dt
                    
                    if (self.lincmd.desired_position_m  > 0.464312):
                        self.lincmd.desired_position_m  = 0.464312
                    elif self.lincmd.desired_position_m  < 0.0:
                        self.lincmd.desired_position_m  = 0.0
                    
                    self.lincmd.header.stamp = rospy.get_rostime()
                    self.lincmd.header.frame_id=''
                    self.linpub.publish(self.lincmd)
                    self.lincmd.header.seq+=1
                self.last_arm_update = rospy.get_time()
                arm_cmd.theta_y = self.axis_value['MAP_CROSS_LR'] * 100.0
                arm_cmd.theta_x = self.axis_value['MAP_CROSS_UD'] * 100.0

                if self.button_state['MAP_lWRIST_LB']:
                    arm_cmd.theta_z = 100.0
                elif self.button_state['MAP_rWRIST_RB']:
                    arm_cmd.theta_z = -100.0

            if self.button_state['MAP_stby_rstick']:
                kg_cmd.data = 5000.0
            elif self.button_state['MAP_TRACT_lstick']:
                kg_cmd.data = -5000.0
            else:
                kg_cmd.data = 0.0
                
            gripper_val =  (self.axis_value['MAP_TRIGGER_LT'] + 1.0)/2.0
            
            if abs(self._last_gripper_val-gripper_val) > 0.05: 
                gripper_cmd.position = gripper_val * 0.085
                gripper_cmd.speed = 0.05
                gripper_cmd.force = 100.0
            
                self.gripper_pub[arm_idx].publish(gripper_cmd)
                self._last_gripper_val = gripper_val
    
            self.arm_pub[arm_idx].publish(arm_cmd)
            self.kgripper_pub[arm_idx].publish(kg_cmd)
        elif self.run_pan_tilt_ctl:
            if self.head_cmd_source[self.head_cmd_current_source_index] == 'teleop':
                vel_cmd = [0.0,0.0]
                if self.axis_value['MAP_TRIGGER_RT']==-1.0:
                    vel_cmd[0]= -self.axis_value['MAP_TWIST_LR_stickleft'] * self.pt_vel_lim
                    vel_cmd[1]= -self.axis_value['MAP_TWIST_UD_stickright'] * self.pt_vel_lim

                self.pt_cmds.pan_cmd.pos_rad += vel_cmd[0] *dt
                self.pt_cmds.tilt_cmd.pos_rad += vel_cmd[1] *dt

                self.pt_cmds.pan_cmd.pos_rad = limit_f(self.pt_cmds.pan_cmd.pos_rad,(math.pi/2.0))
                self.pt_cmds.tilt_cmd.pos_rad = limit_f(self.pt_cmds.tilt_cmd.pos_rad,(math.pi/2.0))

                self.pt_cmds.pan_cmd.vel_rps = 50.0 * (math.pi/180.0)
                self.pt_cmds.tilt_cmd.vel_rps = 50.0 * (math.pi/180.0)
                self.p_pub.publish(self.pt_cmds)
        else:
            if self.button_state['MAP_eSTOP_power']:
                #handled at top of function
                self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_NONE'
                self.cfg_cmd.gp_param = 0  
            elif self.button_state['MAP_stby_rstick']:
                self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
                self.cfg_cmd.gp_param = STANDBY_REQUEST
            elif self.button_state['MAP_TRACT_lstick']:
                self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
                self.cfg_cmd.gp_param = TRACTOR_REQUEST
            else:
                self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_NONE'
                self.cfg_cmd.gp_param = 0
                
            if ('GENERAL_PURPOSE_CMD_NONE' != self.cfg_cmd.gp_cmd):
                self.cfg_cmd.header.stamp = rospy.get_rostime()
                self.cfg_pub.publish(self.cfg_cmd)
                self.cfg_cmd.header.seq
                self.send_cmd_none = True
            elif (True == self.send_cmd_none):
                self.cfg_cmd.header.stamp = rospy.get_rostime()
                self.cfg_pub.publish(self.cfg_cmd)
                self.cfg_cmd.header.seq
                self.send_cmd_none = False
            elif (False == self.send_cmd_none):
                if self.axis_value['MAP_TRIGGER_RT']==-1.0:
                    self.motion_cmd.linear.x =  (self.axis_value['MAP_TWIST_UD_stickleft'] * self.x_vel_limit_mps)
                    self.motion_cmd.linear.y =  (self.axis_value['MAP_TWIST_LR_stickleft'] * self.y_vel_limit_mps)
                    self.motion_cmd.angular.z = (self.axis_value['MAP_TWIST_LR_stickright'] * self.yaw_rate_limit_rps)
                    self.last_motion_command_time = rospy.get_time()
                  
                else:
                    self.motion_cmd.linear.x = 0.0
                    self.motion_cmd.linear.y = 0.0
                    self.motion_cmd.angular.z = 0.0

                if (dt >= 0.01):

                    self.limited_cmd.linear.x = slew_limit(self.motion_cmd.linear.x,
                                                           self.limited_cmd.linear.x,
                                                           self.accel_lim, dt)
                    self.limited_cmd.linear.y = slew_limit(self.motion_cmd.linear.y,
                                                           self.limited_cmd.linear.y,
                                                           self.accel_lim, dt)
                    self.limited_cmd.angular.z = slew_limit(self.motion_cmd.angular.z,
                                                           self.limited_cmd.angular.z,
                                                           self.yaw_accel_lim, dt)

                    if ((rospy.get_time() - self.last_motion_command_time) < 2.0):
         
                        
                        self.motion_pub.publish(self.limited_cmd)
                    
    
