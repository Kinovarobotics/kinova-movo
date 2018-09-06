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

 \Platform: Ubuntu 16.04 LTS / ROS Kinetic
--------------------------------------------------------------------"""
from ctypes import *
import rospy

from movo_msgs.msg import (
    JacoCartesianVelocityCmd,
    JacoAngularVelocityCmd6DOF,
    JacoAngularVelocityCmd7DOF,
    KinovaActuatorFdbk,
    JacoStatus
)
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float32, String
import threading
import math
from angles import *
from helpers import *
from jaco_joint_pid import JacoPID
from kinova_api_wrapper import *
import operator
        
class SIArmController(object):


    # Enum for arm control modes
    # Traectory is the default mode - supports specifying angular or cartesian
    # position trajectories with optional soft velocity and acceleration
    # constraints. A PID controller in this class tracks the setpoints and
    # commands the requisite underlying angular velocities
    TRAJECTORY         = 0

    # Angular velocity mode - angular velocities are passed directly through
    # to the lower layer API
    ANGULAR_VELOCITY   = 1

    # Cartesian velocity mode - cartesian velocities are passed directly
    # through to the lower layer API
    CARTESIAN_VELOCITY = 3


    def __init__(self, prefix="", gripper="", interface='eth0', jaco_ip="10.66.171.15", dof=""):
        """
        Constructor
        """

        # Setup a lock for accessing data in the control loop
        self._lock = threading.Lock()

        # Assume success until posted otherwise
        rospy.loginfo('Starting JACO2 control')
        self.init_success = True
        
        self._prefix = prefix
        self.iface = interface
        self.arm_dof = dof
	self.gripper=gripper

        # List of joint names
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

        # Create the hooks for the API
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
        
        # Initialize the joint feedback
        pos = self.api.get_angular_position()
        vel = self.api.get_angular_velocity()
        force = self.api.get_angular_force()
        self._joint_fb = dict()
        self._joint_fb['position'] = pos[:self._num_joints]
        self._joint_fb['velocity'] = vel[:self._num_joints]
        self._joint_fb['force'] = force[:self._num_joints]
        
        if ("kg2" == gripper) or ("rq85" == gripper):
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
        Reset gravity vector to [0.0 9.81 0.0], along with positive y axis of kinova_arm base
        """
        self.api.set_gravity_vector(0.0, 9.81, 0.0)

        """
        Register the publishers and subscribers
        """
        self.last_cartesian_vel_cmd_update = rospy.get_time()-0.5
        # X, Y, Z, ThetaX, ThetaY, ThetaZ, FingerVel
        self._last_cartesian_vel_cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._cartesian_vel_cmd_sub = rospy.Subscriber(
            "/movo/%s_arm/cartesian_vel_cmd" % self._prefix,
            JacoCartesianVelocityCmd,
            self._update_cartesian_vel_cmd
        )

        self.last_angular_vel_cmd_update = rospy.get_time()-0.5
        self._last_angular_vel_cmd = [0.0] * (self._num_joints + self.num_fingers)
        if "6dof"== self.arm_dof:
            self._angular_vel_cmd_sub = rospy.Subscriber(
                "/movo/%s_arm/angular_vel_cmd" % self._prefix,
                JacoAngularVelocityCmd6DOF,
                self._update_angular_vel_cmd
            )
        elif "7dof" == self.arm_dof:
            self._angular_vel_cmd_sub = rospy.Subscriber(
                "/movo/%s_arm/angular_vel_cmd" % self._prefix,
                JacoAngularVelocityCmd7DOF,
                self._update_angular_vel_cmd
            )
        else:
            # Error condition
            rospy.logerr("DoF needs to be set 6 or 7, was {}".format(self.arm_dof))
            self.Stop()
            return

        self._gripper_vel_cmd = 0.0
        self._ctl_mode = SIArmController.TRAJECTORY
        self.api.set_control_mode(KinovaAPI.ANGULAR_CONTROL)
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
            self._gripper_vel_cmd_sub = rospy.Subscriber("/movo/%s_gripper/vel_cmd"%self._prefix,Float32,self._update_gripper_vel_cmd)
	    if ("rq85" != gripper):
                self._gripper_jspub = rospy.Publisher("/movo/%s_gripper/joint_states"%self._prefix,JointState,queue_size=10)
            self._gripper_jsmsg = JointState()
            self._gripper_jsmsg.header.seq = 0
            self._gripper_jsmsg.header.frame_id = ''
            self._gripper_jsmsg.header.stamp = rospy.get_rostime()
            self._gripper_jsmsg.name  = self._gripper_joint_names

        self._cartesianforce_pub = rospy.Publisher("/movo/%s_arm/cartesianforce"%self._prefix, JacoCartesianVelocityCmd, queue_size=10)
        self._cartesianforce_msg = JacoCartesianVelocityCmd()
        self._cartesianforce_msg.header.seq = 0
        self._cartesianforce_msg.header.frame_id = ''
        self._cartesianforce_msg.header.stamp = rospy.get_rostime()


        self._angularforce_gravityfree_pub = rospy.Publisher("/movo/%s_arm/angularforce_gravityfree"%self._prefix, JacoStatus, queue_size=10)
        self._angularforce_gravityfree_msg = JacoStatus()
        self._angularforce_gravityfree_msg.header.seq = 0
        self._angularforce_gravityfree_msg.header.frame_id = ''
        self._angularforce_gravityfree_msg.header.stamp = rospy.get_rostime()
        self._angularforce_gravityfree_msg.type = "angularforce_gravityfree"

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

        self.pause_controller = False 
                
        self._init_ext_joint_position_control()
        self._init_ext_gripper_control()
        
        # Update the feedback once to get things initialized
        self._update_controller_data()
        
        # Start the controller
        rospy.loginfo("Starting the %s controller"%self._prefix)
        self._done = False
        self._t1 = rospy.Timer(rospy.Duration(0.01),self._run_ctl)
        
    def _init_ext_joint_position_control(self):    
        """
        Initialize the PID controllers, command interface, data processing and
        controller data for the arm
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
        Initialize the PID controllers, command interface, data processing and
        controller data for the gripper
        """        
        if (0 != self.num_fingers):
            for pid in self._gripper_pid:
                pid.initialize()
            self._gripper_pid_error = [0.0]*self.num_fingers
            self._gripper_pid_output = [0.0]*self.num_fingers
            self._gripper_pos_cmds = self._gripper_fb['position']
            self._gripper_vff.Reset(self._gripper_pos_cmds)
            self._gripper_rate_limit.Reset(self._gripper_pos_cmds)
            
    def _update_gripper_vel_cmd(self,cmd):
        self._gripper_vel_cmd = cmd.data

    def _update_cartesian_vel_cmd(self,cmds):
        with self._lock:

            self._last_cartesian_vel_cmd = [
                cmds.x,
                cmds.y,
                cmds.z,
                cmds.theta_x,
                cmds.theta_y,
                cmds.theta_z,
                self._gripper_vel_cmd
            ]
            
            # Switch control mode if needs be
            if (self._ctl_mode != SIArmController.CARTESIAN_VELOCITY):
                self._ctl_mode = SIArmController.CARTESIAN_VELOCITY
                self.api.set_control_mode(KinovaAPI.CARTESIAN_CONTROL)

            # Un-pause the controller
            self.pause_controller = False

            self.last_cartesian_vel_cmd_update = rospy.get_time()

    def _update_angular_vel_cmd(self,cmds):

        with self._lock:

            if "6dof" == self.arm_dof:

                self._last_angular_vel_cmd = [
                    cmds.theta_shoulder_pan_joint,
                    cmds.theta_shoulder_lift_joint,
                    cmds.theta_elbow_joint,
                    cmds.theta_wrist_1_joint,
                    cmds.theta_wrist_2_joint,
                    cmds.theta_wrist_3_joint
                ]

            elif "7dof" == self.arm_dof:
                
                self._last_angular_vel_cmd = [
                    cmds.theta_shoulder_pan_joint,
                    cmds.theta_shoulder_lift_joint,
                    cmds.theta_arm_half_joint,
                    cmds.theta_elbow_joint,
                    cmds.theta_wrist_spherical_1_joint,
                    cmds.theta_wrist_spherical_2_joint,
                    cmds.theta_wrist_3_joint
                ]

            else:
                # Error condition
                rospy.logerr("DoF needs to be set 6 or 7, was {}".format(self.arm_dof))
                self.Stop()
                return

            # Append gripper commands to the cmds list
            for i in range(3):
                self._last_angular_vel_cmd.append(self._gripper_vel_cmd)
            
            # Switch control mode if needs be
            if (self._ctl_mode != SIArmController.ANGULAR_VELOCITY):
                self._ctl_mode = SIArmController.ANGULAR_VELOCITY
                self.api.set_control_mode(KinovaAPI.ANGULAR_CONTROL)

            # Un-pause the controller
            self.pause_controller = False

            self.last_angular_vel_cmd_update = rospy.get_time()
        
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
                self._cartesian_vel_cmd_sub.unregister()
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
        
    def CommandJoints(self, pos, vel=None, acc=None):
        """
        Command the arm with desired joint positions
        Supports soft velocity and acceleration constraints
        """
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
    
    def CommandGripper(self, finger_pos):
        """
        Command the gripper with a desired finger position
        """
        with self._lock:        
            self._gripper_pos_cmds = [finger_pos]*self.num_fingers

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
            self._gripper_pos_cmds = self._gripper_fb['position']    

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
        angular_force = self.api.get_angular_force()
        sensor_data = self.api.get_sensor_data()
        cartesian_force = self.api.get_cartesian_force()
        angular_force_gravity_free = self.api.get_angular_force_gravity_free()

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

        if(len(angular_force) > 0):
            self._joint_fb['force'] = angular_force[:self._num_joints]


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
        

        if (0 != self.num_fingers and "rq85" != self.gripper):
            if (len(pos) > 0):
                self._gripper_fb['position'] = pos[self._num_joints:self._num_joints+self.num_fingers]

            if (len(vel) > 0):
                self._gripper_fb['velocity'] = vel[self._num_joints:self._num_joints+self.num_fingers]

            if (len(angular_force) > 0):
                self._gripper_fb['force'] = angular_force[self._num_joints:self._num_joints+self.num_fingers]
            
            self._gripper_jsmsg.header.stamp = rospy.get_rostime()
            self._gripper_jsmsg.position = self._gripper_fb['position']
            self._gripper_jsmsg.velocity = self._gripper_fb['velocity']
            self._gripper_jsmsg.effort = self._gripper_fb['force']
            self._gripper_jspub.publish(self._gripper_jsmsg)
            self._gripper_jsmsg.header.seq+=1

        # update and publish cartesian force (wrench)
        self._cartesianforce_msg.header.stamp = rospy.get_rostime()
        self._cartesianforce_msg.x = cartesian_force[0]
        self._cartesianforce_msg.y = cartesian_force[1]
        self._cartesianforce_msg.z = cartesian_force[2]
        self._cartesianforce_msg.theta_x = cartesian_force[3]
        self._cartesianforce_msg.theta_y = cartesian_force[4]
        self._cartesianforce_msg.theta_z = cartesian_force[5]

        self._cartesianforce_pub.publish(self._cartesianforce_msg)
        self._cartesianforce_msg.header.seq += 1


        # update and publish angular force gravity free(joint torque)
        self._angularforce_gravityfree_msg.header.stamp = rospy.get_rostime()
        self._angularforce_gravityfree_msg.joint = [round(x, 3) for x in angular_force_gravity_free]
        self._angularforce_gravityfree_pub.publish(self._angularforce_gravityfree_msg)
        self._angularforce_gravityfree_msg.header.seq += 1


    def _run_ctl(self,events):
        if self._is_shutdown():
            return
        
        with self._lock:
            
            # First update the controller data
            self._update_controller_data()

            # Don't do anything if we're e-stopped
            if self.estop:
                return

            if (True == self.pause_controller):
                # If we're paused, don't run any PID calcs, just output zero
                # commands

                self._init_ext_joint_position_control()
                # XXX ajs 19/Mar/2018 Not sure if the gripper should be
                # initialised here as well? Original code didn't. 
                #self._init_ext_gripper_control()

                if (SIArmController.CARTESIAN_VELOCITY == self._ctl_mode):

                    # Send zero cartesian commands
                    # X, Y, Z, ThetaX, ThetaY, ThetaZ, FingerVel
                    self.api.send_cartesian_vel_cmd([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

                elif (SIArmController.ANGULAR_VELOCITY == self._ctl_mode) or\
                    (SIArmController.TRAJECTORY == self._ctl_mode):

                    # Send zero angular commands to all joints and the fingers
                    self.api.send_angular_vel_cmds([0.0] * (self._num_joints + self.num_fingers))

                else:
                    rospy.logerr("{} arm controller: Unrecognized control mode {}".format(
                        self._prefix,
                        self._ctl_mode
                    ))

                return

            if (SIArmController.TRAJECTORY == self._ctl_mode):
                # Handle trajectory control - a PID loop tracks the desired
                # positions (in angular or cartesian space), and computes
                # requisite angular controller velocities

                # Compute the error and update the feedforward terms
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
                
                # Prepare command array
                cmds = self._pid_output

                # Compute finger rates
                if (0 != self.num_fingers):
                    gripper_cmds_lim = self._gripper_rate_limit.Update(self._gripper_pos_cmds)
                    vff = self._gripper_vff.Update(gripper_cmds_lim)
                    self._gripper_pid_error =  map(operator.sub, gripper_cmds_lim, self._gripper_fb['position'])
                    self._gripper_pid_output = [self._gripper_pid[i].compute_output(self._gripper_pid_error[i]) for i in range(self.num_fingers)]
                    self._gripper_pid_output =  map(operator.add, self._gripper_pid_output, vff)                
                    self._gripper_pid_output = [rad_to_deg(limit(self._gripper_pid_output[i],FINGER_ANGULAR_VEL_LIMIT)) for i in range(self.num_fingers)]
                
                # Append gripper commands to the cmds list
                for i in range(3):
                    if (i < self.num_fingers):
                        cmds.append(self._gripper_pid_output[i])
                    else:
                        cmds.append(0.0)

                # Send the command via the API
                self.api.send_angular_vel_cmds(cmds)

                # Finally, publish the angular position controller state
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

            elif (SIArmController.ANGULAR_VELOCITY == self._ctl_mode):
                # Handle angular velocity control - angular velocities are
                # directly passed through to the lower level controller

                self._init_ext_joint_position_control()
                self._init_ext_gripper_control()

                # Safety check: If it has been more than 1 second since
                # the last command, drop back to trajectory control and
                # pause the controller
                if ((rospy.get_time() - self.last_angular_vel_cmd_update) >= 1.0):
                    self._ctl_mode = SIArmController.TRAJECTORY
                    self.api.set_control_mode(KinovaAPI.ANGULAR_CONTROL)
                    self.pause_controller = True
                    return
                
                # Safety check: If it has been more than 0.5 seconds since
                # the last command, zero the velocities
                if ((rospy.get_time() - self.last_angular_vel_cmd_update) >= 0.5):
                    self._last_angular_vel_cmd = [0.0] * (self._num_joints + self.num_fingers)

                # Safety check: Apply rate limits for arm joints
                cmd_limited = self._last_angular_vel_cmd
                for joint in range(self._num_joints):

                    if "6dof" == self.arm_dof:
                        cmd_limited[joint] = rad_to_deg(
                            limit(
                                deg_to_rad(cmd_limited[joint]),
                                JOINT_6DOF_VEL_LIMITS[joint]
                            )
                        )
                    elif "7dof" == self.arm_dof:
                        cmd_limited[joint] = rad_to_deg(
                            limit(
                                deg_to_rad(cmd_limited[joint]),
                                JOINT_7DOF_VEL_LIMITS[joint]
                            )
                        )
                    else:
                        # Error condition
                        rospy.logerr("DoF needs to be set 6 or 7, was {}".format(self.arm_dof))
                        self.Stop()
                        return

                # Safety check: Apply rate limits for finger joints
                for finger in range(self.num_fingers):
                    cmd_limited[self._num_joints + finger] = rad_to_deg(
                        limit(
                            deg_to_rad(cmd_limited[self._num_joints + finger]),
                            FINGER_ANGULAR_VEL_LIMIT
                        )
                    )

                # Command angular velocities
                self.api.send_angular_vel_cmds(cmd_limited)

            elif (SIArmController.CARTESIAN_VELOCITY == self._ctl_mode):
                # Handle cartesian velocity control mode - cartesian
                # velocities are passed through to the lower level controller
                # after some basic safety / timeout checks

                self._init_ext_joint_position_control()
                self._init_ext_gripper_control()

                # Safety check: If it has been more than 1 second since
                # the last command, drop back to angular position control
                if ((rospy.get_time() - self.last_cartesian_vel_cmd_update) >= 1.0):
                    self._ctl_mode = SIArmController.TRAJECTORY
                    self.api.set_control_mode(KinovaAPI.ANGULAR_CONTROL)
                    self.pause_controller = True
                    return
                
                # Safety check: If it has been more than 0.5 seconds since
                # the last command, zero the velocities
                if ((rospy.get_time() - self.last_cartesian_vel_cmd_update) >= 0.5):
                    self._last_cartesian_vel_cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                # Command cartesian velocities
                self.api.send_cartesian_vel_cmd(self._last_cartesian_vel_cmd)

            else:
                # Unknown control mode
                rospy.logerr("{} arm controller: Unrecognized control mode {}".format(
                    self._prefix,
                    self._ctl_mode
                ))

            return
