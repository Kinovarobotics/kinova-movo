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
 
 \file   jaco_joint_controller.py

 \brief  This module contains a collection of functions low level interface
         to the Kinova API.

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from ctypes import *
import rospy
from movo_msgs.msg import JacoCartesianVelocityCmd,KinovaActuatorFdbk
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float32
import threading
import math
from angles import *
from helpers import *
from jaco_joint_pid import JacoPID
from kinova_api_wrapper import *
import operator
        
class SIArmController(object):
    def __init__(self, prefix="", gripper="", interface='eth0', jaco_ip="10.66.171.15", dof=""):
        """
        Setup a lock for accessing data in the control loop
        """
        self._lock = threading.Lock()

        """
        Assume success until posted otherwise
        """
        rospy.loginfo('Starting JACO2 control')
        self.init_success = True
        
        self._prefix = prefix
        self.iface = interface
        self.arm_dof = dof

        """
        List of joint names
        """
        if ("6dof"== self.arm_dof):
            self._joint_names = [self._prefix+'_shoulder_pan_joint',
                                 self._prefix+'_shoulder_lift_joint',
                                 self._prefix+'_elbow_joint',
                                 self._prefix+'_wrist_1_joint',
                                 self._prefix+'_wrist_2_joint',
                                 self._prefix+'_wrist_3_joint']
        elif ("7dof"== self.arm_dof):
            self._joint_names = [self._prefix + '_shoulder_pan_joint',
                                 self._prefix + '_shoulder_lift_joint',
                                 self._prefix + '_arm_half_joint',
                                 self._prefix + '_elbow_joint',
                                 self._prefix + '_wrist_spherical_1_joint',
                                 self._prefix + '_wrist_spherical_2_joint',
                                 self._prefix + '_wrist_3_joint']

        else:
            rospy.logerr("DoF needs to be set 6 or 7, cannot start SIArmController")
            return
                             
        self._num_joints = len(self._joint_names)

        """
        Create the hooks for the API
        """
        if ('left' == prefix):
            self.api = KinovaAPI('left',self.iface,jaco_ip,'255.255.255.0',24000,24024,44000, self.arm_dof)
        elif ('right' == prefix):
            self.api = KinovaAPI('right',self.iface,jaco_ip,'255.255.255.0',25000,25025,55000, self.arm_dof)
        else:
            rospy.logerr("prefix needs to be set to left or right, cannot start the controller")
            return
        
        if not (self.api.init_success):
            self.Stop()
            return
        
        self.api.SetCartesianControl()
        self._position_hold = False
        self.estop = False
        
        """
        Initialize the joint feedback
        """
        pos = self.api.get_angular_position()
        vel = self.api.get_angular_velocity()
        force = self.api.get_angular_force()
        self._joint_fb = dict()
        self._joint_fb['position'] = pos[:self._num_joints]
        self._joint_fb['velocity'] = vel[:self._num_joints]
        self._joint_fb['force'] = force[:self._num_joints]

        
        if ("kg2" == gripper):
            self._gripper_joint_names = [self._prefix+'_gripper_finger1_joint',
                                         self._prefix+'_gripper_finger2_joint']
            self.num_fingers = 2
        elif ("kg3" == gripper):
            self._gripper_joint_names = [self._prefix+'_gripper_finger1_joint',
                                         self._prefix+'_gripper_finger2_joint',
                                         self._prefix+'_gripper_finger3_joint']
            self.num_fingers = 3
        
        if (0 != self.num_fingers):
            self._gripper_fb = dict()
            self._gripper_fb['position'] = pos[self._num_joints:self._num_joints+self.num_fingers]
            self._gripper_fb['velocity'] = vel[self._num_joints:self._num_joints+self.num_fingers]
            self._gripper_fb['force'] = force[self._num_joints:self._num_joints+self.num_fingers]
                
        """
        Register the publishers and subscribers
        """
        self.last_teleop_cmd_update = rospy.get_time()-0.5
        self._teleop_cmd_sub = rospy.Subscriber("/movo/%s_arm/cartesian_vel_cmd"%self._prefix,JacoCartesianVelocityCmd,self._update_teleop_cmd)
        
        self._gripper_cmd = 0.0
        self._ctl_mode = AUTONOMOUS_CONTROL
        self._jstpub = rospy.Publisher("/movo/%s_arm_controller/state"%self._prefix,JointTrajectoryControllerState,queue_size=10)
        self._jstmsg = JointTrajectoryControllerState()
        self._jstmsg.header.seq = 0
        self._jstmsg.header.frame_id = ''
        self._jstmsg.header.stamp = rospy.get_rostime() 
        self._jspub = rospy.Publisher("/movo/%s_arm/joint_states"%self._prefix,JointState,queue_size=10)
        self._jsmsg = JointState()
        self._jsmsg.header.seq = 0
        self._jsmsg.header.frame_id = ''
        self._jsmsg.header.stamp = rospy.get_rostime()
        self._jsmsg.name  = self._joint_names
        
        self._actfdbk_pub = rospy.Publisher("/movo/%s_arm/actuator_feedback"%self._prefix,KinovaActuatorFdbk,queue_size=10)
        self._actfdbk_msg = KinovaActuatorFdbk()
        self._jsmsg.header.seq = 0
        self._jsmsg.header.frame_id = ''
        self._jsmsg.header.stamp = rospy.get_rostime()

        if (0 != self.num_fingers):
            self._teleop_gripper_cmd_sub = rospy.Subscriber("/movo/%s_gripper/vel_cmd"%self._prefix,Float32,self._update_teleop_gripper_cmd)
            self._gripper_jspub = rospy.Publisher("/movo/%s_gripper/joint_states"%self._prefix,JointState,queue_size=10)
            self._gripper_jsmsg = JointState()
            self._gripper_jsmsg.header.seq = 0
            self._gripper_jsmsg.header.frame_id = ''
            self._gripper_jsmsg.header.stamp = rospy.get_rostime()
            self._gripper_jsmsg.name  = self._gripper_joint_names
        
        """
        This starts the controller in cart vel mode so that teleop is active by default
        """
        if (0 != self.num_fingers):
            self._gripper_pid = [None]*self.num_fingers
            for i in range(self.num_fingers):
                self._gripper_pid[i] = JacoPID(5.0,0.0,0.8)
            self._gripper_vff = DifferentiateSignals(self.num_fingers, self._gripper_fb['position'])
            self._gripper_rate_limit = RateLimitSignals([FINGER_ANGULAR_VEL_LIMIT]*self.num_fingers,self.num_fingers,self._gripper_fb['position'])

        if ("6dof" == self.arm_dof):
            self._arm_rate_limit = RateLimitSignals(JOINT_6DOF_VEL_LIMITS,self._num_joints,self._joint_fb['position'])

        if ("7dof" == self.arm_dof):
            self._arm_rate_limit = RateLimitSignals(JOINT_7DOF_VEL_LIMITS, self._num_joints, self._joint_fb['position'])

        self._arm_vff_diff = DifferentiateSignals(self._num_joints, self._joint_fb['position'])        

        self._pid = [None]*self._num_joints

        for i in range(self._num_joints):
            self._pid[i] = JacoPID(5.0,0.0,0.8)

        """
        self._pid[0] = JacoPID(5.0,0.0,0.8)
        self._pid[1] = JacoPID(5.0,0.0,0.8)
        self._pid[2] = JacoPID(5.0,0.0,0.8)
        self._pid[3] = JacoPID(5.0,0.0,0.8)
        self._pid[4] = JacoPID(5.0,0.0,0.8) 
        self._pid[5] = JacoPID(5.0,0.0,0.8)
        """

        self.pause_controller = False 
                
        self._init_ext_joint_position_control()
        self._init_ext_gripper_control()
        
        """
        Set temporary tucked position after homing
        """
        """
        if 'left' == self._prefix:
            self._arm_cmds['position'][1]+= deg_to_rad(80.0)
            self._arm_cmds['position'][2]+= deg_to_rad(50.0)
            self._arm_cmds['position'][4]-= deg_to_rad(90.0)
        else:
            self._arm_cmds['position'][1]-= deg_to_rad(80.0)
            self._arm_cmds['position'][2]-= deg_to_rad(50.0)
            self._arm_cmds['position'][4]+= deg_to_rad(90.0)
        """
        
        """
        Update the feedback once to get things initialized
        """
        self._update_controller_data()
        
        """
        Start the controller
        """ 
        rospy.loginfo("Starting the %s controller"%self._prefix)
        self._done = False
        self._t1 = rospy.Timer(rospy.Duration(0.01),self._run_ctl)
        
    def _init_ext_joint_position_control(self):    
        """
        Initialize the PID controllers, command interface, data processing and controller data
        for the arm
        """
        for pid in self._pid:
            pid.initialize()    
        self._pid_error = [0.0]*self._num_joints
        self._pid_output = [0.0]*self._num_joints               
        self._arm_cmds = dict()
        self._arm_cmds['position'] = self._joint_fb['position']
        self._arm_cmds['velocity'] = [0.0]*self._num_joints
        self._arm_cmds['acceleration'] = [0.0]*self._num_joints
        self._arm_rate_limit.Reset(self._arm_cmds['position'])
        self._arm_vff_diff.Reset(self._arm_cmds['position'])        

    def _init_ext_gripper_control(self):
        """
        Initialize the PID controllers, command interface, data processing and controller data
        for the gripper
        """        
        if (0 != self.num_fingers):
            for pid in self._gripper_pid:
                pid.initialize()
            self._gripper_pid_error = [0.0]*self.num_fingers
            self._gripper_pid_output = [0.0]*self.num_fingers
            self._gripper_cmds = self._gripper_fb['position']
            self._gripper_vff.Reset(self._gripper_cmds)
            self._gripper_rate_limit.Reset(self._gripper_cmds) 

    def _update_teleop_cmd(self,cmds):
        with self._lock:
            self.api.update_cartesian_vel_cmd([cmds.x,cmds.y,cmds.z,cmds.theta_x,cmds.theta_y,cmds.theta_z,self._gripper_cmd])
            if (self._ctl_mode != TELEOP_CONTROL):
                self.api.set_control_mode(TELEOP_CONTROL)
                self._ctl_mode = TELEOP_CONTROL
            self.last_teleop_cmd_update = rospy.get_time()
            
    def _update_teleop_gripper_cmd(self,cmd):
        self._gripper_cmd = cmd.data
        
    def SetEstop(self):
        self._init_ext_joint_position_control()
        self.estop = True

    def ClearEstop(self):
        self.estop = False

    def Stop(self):
        rospy.loginfo("Stopping the %s arm controller"%self._prefix)
        with self._lock:
            try:
                self._t1.shutdown()
            except:
                pass
            try:
                self._jspub.unregister()
                self._teleop_cmd_sub.unregister()
                self._jspub.unregister()
            except:
                pass
            self.api.Shutdown()
            
            rospy.loginfo("%s arm controller has stopped"%self._prefix)
            self._done = True

    def _is_shutdown(self):
        if rospy.is_shutdown():
            self.Stop()
        return self._done
        
    def UpdatePIDGains(self,pid_gains):
        new_pid_gains = [pid_gains[jnt] for jnt in self._joint_names]
        
    def Pause(self):
        self.pause_controller = True
        
    def Resume(self):
        self.pause_controller = False
        
        
    def GetCtlStatus(self):
        return self.api.api_online
        
    def SetPositionHold(self):
        if self._position_hold:
            return
        with self._lock:
            self._position_hold=True
            self._arm_cmds['position'] = self._joint_fb['position']
            self._arm_cmds['velocity'] = [0.0]*self._num_joints
            self._arm_cmds['acceleration'] = [0.0]*self._num_joints
            
    def ClearPositionHold(self):
        with self._lock:
            self._position_hold=False
        
    def CommandJoints(self,pos,vel=None,acc=None):
        if self._position_hold:
            return False

        with self._lock:
            self._arm_cmds['position'] = [pos[jnt] for jnt in self._joint_names]
            tmp = [i for i in self._arm_cmds['position']]
            for jnt in range(self._num_joints):
                if ("6dof" == self.arm_dof):
                    if (jnt!=1) and (jnt!=2):
                        self._arm_cmds['position'][jnt] = get_smallest_difference_to_cont_angle(tmp[jnt],self._joint_fb['position'][jnt])
                if ("7dof" == self.arm_dof):
                    if (jnt!=1) and (jnt!=3) and (jnt!=5):
                        self._arm_cmds['position'][jnt] = get_smallest_difference_to_cont_angle(tmp[jnt],self._joint_fb['position'][jnt])
            
            if vel:
                self._arm_cmds['velocity'] = [vel[jnt] for jnt in self._joint_names]
            else:
                self._arm_cmds['velocity'] = [0.0]*self._num_joints    
            if acc:
                self._arm_cmds['acceleration'] = [acc[jnt] for jnt in self._joint_names]
            else:
                self._arm_cmds['acceleration'] = [0.0]*self._num_joints
                
        return True
    
    def CommandGripper(self,finger_pos):
        with self._lock:        
            self._gripper_cmds = [finger_pos]*self.num_fingers                    

    def GetGripperFdbk(self):
        gripperfdbk = [0]*3        
        with self._lock:

            gripperfdbk[0] = self._gripper_fb['position']
            gripperfdbk[1] = self._gripper_fb['velocity']
            tmp = self._actfdbk_msg.current[self._num_joints:self._num_joints+self.num_fingers]
            gripperfdbk[2] = [(i/0.8) * 25 for i in tmp]
            
        return gripperfdbk
    
    def StopGripper(self):
        with self._lock:
            self._gripper_cmds = self._gripper_fb['position']    

    def GetCurrentJointPosition(self, joint_names):
        with self._lock:
            pos = dict(zip(self._jsmsg.name,self._joint_fb['position']))
        pos = [pos[jnt] for jnt in joint_names]
        return pos
        
    def GetCurrentJointVelocity(self,joint_names):
        with self._lock:
            vel = dict(zip(self._jsmsg.name,self._joint_fb['velocity']))
        vel = [vel[jnt] for jnt in joint_names]
        return vel

    def GetCurrentJointPositionError(self,joint_names):
        with self._lock:
            pos_error = dict(zip(self._jsmsg.name,self._pid_error))
        pos_error = [pos_error[jnt] for jnt in joint_names]
        return pos_error  
         
    def _update_controller_data(self):
        pos = self.api.get_angular_position()
        vel = self.api.get_angular_velocity()
        force = self.api.get_angular_force()
        sensor_data = self.api.get_sensor_data()

        if(len(sensor_data[0]) > 0):
            self._actfdbk_msg.current = sensor_data[0]
            
        if(len(sensor_data[1]) > 0):
            self._actfdbk_msg.temperature = sensor_data[1]

        self._actfdbk_msg.header.stamp = rospy.get_rostime()
        self._actfdbk_msg.header.seq+=1
        self._actfdbk_pub.publish(self._actfdbk_msg)

        if(len(pos) > 0):
            self._joint_fb['position'] = pos[:self._num_joints]

        if(len(vel) > 0):
            self._joint_fb['velocity'] = vel[:self._num_joints]

        if(len(force) > 0):
            self._joint_fb['force'] = force[:self._num_joints]


        tmp = [0.0]*self._num_joints
        if ("6dof"== self.arm_dof):
            tmp[0] = wrap_angle(self._joint_fb['position'][0])
            tmp[1] = self._joint_fb['position'][1]
            tmp[2] = self._joint_fb['position'][2]
            tmp[3] = wrap_angle(self._joint_fb['position'][3])
            tmp[4] = wrap_angle(self._joint_fb['position'][4])
            tmp[5] = wrap_angle(self._joint_fb['position'][5])

        if("7dof"== self.arm_dof):
            tmp[0] = wrap_angle(self._joint_fb['position'][0])
            tmp[1] = self._joint_fb['position'][1]
            tmp[2] = wrap_angle(self._joint_fb['position'][2])
            tmp[3] = self._joint_fb['position'][3]
            tmp[4] = wrap_angle(self._joint_fb['position'][4])
            tmp[5] = self._joint_fb['position'][5]
            tmp[6] = wrap_angle(self._joint_fb['position'][6])

        self._jsmsg.header.stamp = rospy.get_rostime()
        self._jsmsg.position = tmp
        self._jsmsg.velocity = self._joint_fb['velocity']
        self._jsmsg.effort = self._joint_fb['force']
        self._jspub.publish(self._jsmsg)
        self._jsmsg.header.seq+=1
        

        if (0 != self.num_fingers):
            if (len(pos) > 0):
                self._gripper_fb['position'] = pos[self._num_joints:self._num_joints+self.num_fingers]

            if (len(vel) > 0):
                self._gripper_fb['velocity'] = vel[self._num_joints:self._num_joints+self.num_fingers]

            if (len(force) > 0):
                self._gripper_fb['force'] = force[self._num_joints:self._num_joints+self.num_fingers]
            
            self._gripper_jsmsg.header.stamp = rospy.get_rostime()
            self._gripper_jsmsg.position = self._gripper_fb['position']
            self._gripper_jsmsg.velocity = self._gripper_fb['velocity']
            self._gripper_jsmsg.effort = self._gripper_fb['force']
            self._gripper_jspub.publish(self._gripper_jsmsg)
            self._gripper_jsmsg.header.seq+=1

    def _run_ctl(self,events):
        if self._is_shutdown():
            return
        
        with self._lock:
            
            """
            First update the controller data
            """
            self._update_controller_data()
            
            if self.estop:
                return
        
            if (TELEOP_CONTROL == self._ctl_mode):
                self._init_ext_joint_position_control()
                self._init_ext_gripper_control()
                if ((rospy.get_time() - self.last_teleop_cmd_update) >= 1.0):
                    self.api.set_control_mode(AUTONOMOUS_CONTROL)
                    self._ctl_mode = AUTONOMOUS_CONTROL
                    return
                
                if ((rospy.get_time() - self.last_teleop_cmd_update) >= 0.5):
                    self.api.update_cartesian_vel_cmd([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
                self.api.send_cartesian_vel_cmd()
                return
            
            if (True == self.pause_controller):
                self._init_ext_joint_position_control()
                cmds = [0.0] * self._num_joints
            else:

                """
                Compute the error and update the feedforward terms
                """
                arm_cmds_lim = self._arm_rate_limit.Update(self._arm_cmds['position'])
                vff = self._arm_vff_diff.Update(arm_cmds_lim)
                scaled_ff_vel = map(operator.mul, vff, [1.0] * self._num_joints)
                scaled_ff_acc = map(operator.mul, self._arm_cmds['acceleration'], [0.0] * self._num_joints)
                ff_terms = map(operator.add, scaled_ff_vel, scaled_ff_acc)
            
                self._pid_error =  map(operator.sub, arm_cmds_lim, self._joint_fb['position'])
                self._pid_output  = [self._pid[i].compute_output(self._pid_error[i]) for i in range(self._num_joints)]
                self._pid_output = map(operator.add,self._pid_output, ff_terms)
                if ("6dof" == self.arm_dof):
                    self._pid_output = [rad_to_deg(limit(self._pid_output[i],JOINT_6DOF_VEL_LIMITS[i])) for i in range(self._num_joints)]
                if ("7dof" == self.arm_dof):
                    self._pid_output = [rad_to_deg(limit(self._pid_output[i],JOINT_7DOF_VEL_LIMITS[i])) for i in range(self._num_joints)]
            
                """
                Send the command via the API
                """
                cmds = self._pid_output

            if (0 != self.num_fingers):
                gripper_cmds_lim = self._gripper_rate_limit.Update(self._gripper_cmds)
                vff = self._gripper_vff.Update(gripper_cmds_lim)
                self._gripper_pid_error =  map(operator.sub, gripper_cmds_lim, self._gripper_fb['position'])
                self._gripper_pid_output = [self._gripper_pid[i].compute_output(self._gripper_pid_error[i]) for i in range(self.num_fingers)]
                self._gripper_pid_output =  map(operator.add, self._gripper_pid_output, vff)                
                self._gripper_pid_output = [rad_to_deg(limit(self._gripper_pid_output[i],FINGER_ANGULAR_VEL_LIMIT)) for i in range(self.num_fingers)]
            
            for i in range(3):
                if (i < self.num_fingers):
                    cmds.append(self._gripper_pid_output[i])
                else:
                    cmds.append(0.0)

            self.api.send_angular_vel_cmds(cmds)
            
            """
            Publish the controller state
            """    
            self._jstmsg.header.frame_id = ''
            self._jstmsg.header.stamp = rospy.get_rostime()
            self._jstmsg.desired.positions=self._arm_cmds['position']
            self._jstmsg.desired.velocities=self._arm_cmds['velocity']
            self._jstmsg.desired.accelerations=self._arm_cmds['acceleration']
            self._jstmsg.actual.positions=self._joint_fb['position']
            self._jstmsg.actual.velocities=self._joint_fb['velocity']
            self._jstmsg.actual.accelerations=[0.0]*self._num_joints
            self._jstmsg.error.positions = self._pid_error
            self._jstmsg.error.velocities= map(operator.sub, self._arm_cmds['velocity'], self._joint_fb['velocity']) 
            self._jstmsg.error.accelerations=[0.0]*self._num_joints                
            self._jstpub.publish(self._jstmsg) 
            self._jstmsg.header.seq +=1                       
     
