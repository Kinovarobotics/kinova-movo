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
 
 \file   kinova_api_wrapper.py

 \brief  This module contains a collection of functions low level interface
         to the Kinova API in order to control an arm.

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from ctypes import *
from helpers import dottedQuadToNum,get_ip_address
from angles import deg_to_rad, rad_to_deg
import rospy

class KinovaDevice(Structure):
    _fields_=[("SerialNumber",c_char*20),
              ("Model",c_char*20),
              ("VersionMajor",c_int),
              ("VersionMinor",c_int),
              ("VersionRelease",c_int),
              ("DeviceType",c_int),
              ("DeviceID",c_int)]
    
class EthernetCommConfig(Structure):
    _fields_=[("localIpAddress",c_ulong),
              ("subnetMask",c_ulong),
              ("robotIpAddress",c_ulong),
              ("localCmdport",c_ushort),
              ("localBcastPort",c_ushort),
              ("robotPort",c_ushort),
              ("rxTimeOutInMs",c_ulong)]
    
class AngularInfo(Structure):
    _fields_=[("Actuator1",c_float),
              ("Actuator2",c_float),
              ("Actuator3",c_float),
              ("Actuator4",c_float),
              ("Actuator5",c_float),
              ("Actuator6",c_float),
              ("Actuator7",c_float)]
              
class FingersPosition(Structure):
    _fields_=[("Finger1",c_float),
              ("Finger2",c_float),
              ("Finger3",c_float)]
    
class AngularPosition(Structure):
    _fields_=[("Actuators",AngularInfo),
              ("Fingers",FingersPosition)]
    
class Limitation(Structure):
    _fields_ = [("speedParameter1",c_float),
                ("speedParameter2",c_float),
                ("speedParameter3",c_float),
                ("forceParameter1",c_float),
                ("forceParameter2",c_float),
                ("forceParameter3",c_float),
                ("accelerationParameter1",c_float),
                ("accelerationParameter2",c_float),
                ("accelerationParameter3",c_float)]
    
class CartesianInfo(Structure):
    _fields_ = [("X",c_float),
                ("Y",c_float),
                ("Z",c_float),
                ("ThetaX",c_float),
                ("ThetaY",c_float),
                ("ThetaZ",c_float)]
    
class UserPosition(Structure):
    _fields_ = [("Type",c_int),
                ("Delay",c_float),
                ("CartesianPosition",CartesianInfo),
                ("Actuators",AngularInfo),
                ("HandMode",c_int),
                ("Fingers",FingersPosition)]
    
class TrajectoryPoint(Structure):
    _fields_ = [("Position",UserPosition),
                ("LimitationsActive",c_int),
                ("SynchroType",c_int),
                ("Limitations",Limitation)]
    
class SensorInfo(Structure):
    _fields_ = [("Voltage",c_float),
                ("Current",c_float),
                ("AccelerationX",c_float),
                ("AccelerationY",c_float),
                ("AccelerationZ",c_float),
                ("ActuatorTemp1",c_float),
                ("ActuatorTemp2",c_float),
                ("ActuatorTemp3",c_float),
                ("ActuatorTemp4",c_float),
                ("ActuatorTemp5",c_float),
                ("ActuatorTemp6",c_float),
                ("ActuatorTemp7",c_float),
                ("FingerTemp1",c_float),
                ("FingerTemp2",c_float),
                ("FingerTemp3",c_float)]
    
NO_ERROR_KINOVA = 1
LARGE_ACTUATOR_VELOCITY =deg_to_rad(35.0) #maximum velocity of large actuator (joints 1-3) (deg/s)
SMALL_ACTUATOR_VELOCITY =deg_to_rad(45.0) #maximum velocity of small actuator (joints 4-6) (deg/s)
ANGULAR_POSITION   = 2
CARTESIAN_VELOCITY = 7
ANGULAR_VELOCITY   = 8

FINGER_FACTOR = (0.986111027/121.5)

LARGE_ACTUATOR_VELOCITY =deg_to_rad(35.0) #maximum velocity of large actuator (joints 1-3) (deg/s)
SMALL_ACTUATOR_VELOCITY =deg_to_rad(45.0) #maximum velocity of small actuator (joints 4-6) (deg/s)

JOINT_6DOF_VEL_LIMITS = [LARGE_ACTUATOR_VELOCITY,
                         LARGE_ACTUATOR_VELOCITY,
                         LARGE_ACTUATOR_VELOCITY,
                         SMALL_ACTUATOR_VELOCITY,
                         SMALL_ACTUATOR_VELOCITY,
                         SMALL_ACTUATOR_VELOCITY]

JOINT_7DOF_VEL_LIMITS = [LARGE_ACTUATOR_VELOCITY,
                         LARGE_ACTUATOR_VELOCITY,
                         LARGE_ACTUATOR_VELOCITY,
                         LARGE_ACTUATOR_VELOCITY,
                         SMALL_ACTUATOR_VELOCITY,
                         SMALL_ACTUATOR_VELOCITY,
                         SMALL_ACTUATOR_VELOCITY]

FINGER_ANGULAR_VEL_LIMIT = deg_to_rad(4500.0)*FINGER_FACTOR

AUTONOMOUS_CONTROL   = 0
TELEOP_CONTROL       = 1


class KinovaAPI(object):
    def __init__(self, prefix, interface='eth0', robotIpAddress="10.66.171.15", subnetMask="255.255.255.0", localCmdport = 24000,localBcastPort = 24024,robotPort = 44000, dof="6dof"):
        
        self.init_success = False
        self.api_online = False
        self._prefix = prefix
        self.commErrCnt = 0
        self.arm_dof = dof
        
        """
        Create the hooks for the API
        """
        self.kinova=CDLL('Kinova.API.EthCommandLayerUbuntu.so')
        self.InitAPI = self.kinova.Ethernet_InitEthernetAPI
        self.InitAPI.arg_types = [POINTER(EthernetCommConfig)]
        self.CloseAPI = self.kinova.Ethernet_CloseAPI
        self.RefresDevicesList = self.kinova.Ethernet_RefresDevicesList
        self.GetDevices = self.kinova.Ethernet_GetDevices
        self.GetDevices.argtypes = [POINTER(KinovaDevice),POINTER(c_int)]
        self.SetActiveDevice = self.kinova.Ethernet_SetActiveDevice
        self.SetActiveDevice.argtypes = [KinovaDevice]
        self.GetAngularPosition = self.kinova.Ethernet_GetAngularPosition
        self.GetAngularPosition.argtypes = [POINTER(AngularPosition)]
        self.GetAngularVelocity = self.kinova.Ethernet_GetAngularVelocity
        self.GetAngularVelocity.argtypes = [POINTER(AngularPosition)]
        self.GetAngularForce = self.kinova.Ethernet_GetAngularForce
        self.GetAngularForce.argtypes = [POINTER(AngularPosition)]
        self.GetSensorsInfo = self.kinova.Ethernet_GetSensorsInfo
        self.GetSensorsInfo.argtypes=[POINTER(SensorInfo)]
        self.GetAngularCurrentMotor = self.kinova.Ethernet_GetAngularCurrentMotor
        self.GetAngularCurrentMotor.argtypes = [POINTER(AngularPosition)]
        self.StartControlAPI = self.kinova.Ethernet_StartControlAPI
        self.StopControlAPI = self.kinova.Ethernet_StopControlAPI
        self.SendAdvanceTrajectory = self.kinova.Ethernet_SendAdvanceTrajectory
        self.SendAdvanceTrajectory.argtypes = [TrajectoryPoint]
        self.SendBasicTrajectory = self.kinova.Ethernet_SendBasicTrajectory
        self.SendBasicTrajectory.argtypes = [TrajectoryPoint]
        self.EraseAllTrajectories = self.kinova.Ethernet_EraseAllTrajectories
        self.SetAngularControl = self.kinova.Ethernet_SetAngularControl
        self.SetCartesianControl = self.kinova.Ethernet_SetCartesianControl
        self.MoveHome = self.kinova.Ethernet_MoveHome
        self.InitFingers = self.kinova.Ethernet_InitFingers
        self.DevInfoArrayType = ( KinovaDevice * 20 )
        
        local_ip = get_ip_address(interface)
        eth_cfg = EthernetCommConfig()
        eth_cfg.localIpAddress = dottedQuadToNum(local_ip)
        eth_cfg.subnetMask = dottedQuadToNum(subnetMask)
        eth_cfg.robotIpAddress = dottedQuadToNum(robotIpAddress)
        eth_cfg.localCmdport = localCmdport
        eth_cfg.localBcastPort = localBcastPort
        eth_cfg.robotPort = robotPort
        eth_cfg.rxTimeOutInMs = 1000
        
        result1 = self.InitAPI(byref(eth_cfg))
        self.RefresDevicesList()
        result2 = c_int(0)
        devinfo = self.DevInfoArrayType()
        num_arms = self.GetDevices(devinfo,byref(result2))

        if (NO_ERROR_KINOVA != result1) or (NO_ERROR_KINOVA != result2.value) or (num_arms <= 0):
            rospy.logerr("Init API result:   %d"%result1) 
            rospy.logerr("GetDevices result: %d"%result2.value)
            rospy.logerr("Number of arms:    %d"%num_arms)
            rospy.logerr("Initialization failed, could not find Kinova devices \
                          (see Kinova.API.CommLayerUbuntu.h for details)")
            self.CloseAPI()
            return
        
        """
        Collect the information for the arm since we know there is one
        """
        self.sn = devinfo[0].SerialNumber
        self._arm = devinfo[0]
        rospy.loginfo('%s arm at %s has serial number: %s'%(self._prefix,robotIpAddress,self.sn))

        """
        Try and set the active device, the API version not matching is usually why this fails
        """
        result = self.SetActiveDevice(self._arm)
        result &= self.SetCartesianControl()
        if not (NO_ERROR_KINOVA == result):
            rospy.logerr("Could not set %s arm active...stopping the driver"%self._prefix)
            rospy.logerr("Set Active Device result:   %d"%result) 
            rospy.logerr("Initialization failed, could not find Kinova devices \
                          (see Kinova.API.CommLayerUbuntu.h for details)")
            self.Shutdown()
            return
            
        self.api_online = True
        
        self._cart_cmd = TrajectoryPoint()
        self._cart_cmd.Position.Type = CARTESIAN_VELOCITY
        self.init_success = True
    
    def Shutdown(self):
        self.StopControlAPI()
        self.CloseAPI()
    
    def set_control_mode(self,mode):
        if (AUTONOMOUS_CONTROL == mode):
            self.SetAngularControl()
        elif (TELEOP_CONTROL == mode):
            self.SetCartesianControl()

    def handle_comm_err(self, err):
        if (NO_ERROR_KINOVA == err):
            self.api_online = True
            self.commErrCnt = 0
        else:
            self.commErrCnt += 1
            if (self.commErrCnt > 5):
                self.api_online = False
    
    def update_cartesian_vel_cmd(self,cmds):
        self._cart_cmd.Position.CartesianPosition.X = cmds[0]
        self._cart_cmd.Position.CartesianPosition.Y = cmds[1]
        self._cart_cmd.Position.CartesianPosition.Z = cmds[2]
        self._cart_cmd.Position.CartesianPosition.ThetaX = cmds[3]
        self._cart_cmd.Position.CartesianPosition.ThetaY = cmds[4]
        self._cart_cmd.Position.CartesianPosition.ThetaZ = cmds[5]
        self._cart_cmd.Position.HandMode = 2
        self._cart_cmd.Position.Fingers.Finger1 = cmds[6]/FINGER_FACTOR
        self._cart_cmd.Position.Fingers.Finger2 = cmds[6]/FINGER_FACTOR
        self._cart_cmd.Position.Fingers.Finger3 = cmds[6]/FINGER_FACTOR
        
    def send_angular_vel_cmds(self,cmds):
        cmds_len = len(cmds)
        traj = TrajectoryPoint()
        traj.Position.Type = ANGULAR_VELOCITY
        if ("6dof" == self.arm_dof):
            traj.Position.Actuators.Actuator1 = cmds[0]
            traj.Position.Actuators.Actuator2 = cmds[1]
            traj.Position.Actuators.Actuator3 = cmds[2]
            traj.Position.Actuators.Actuator4 = cmds[3]
            traj.Position.Actuators.Actuator5 = cmds[4]
            traj.Position.Actuators.Actuator6 = cmds[5]
            traj.Position.HandMode = 2
            traj.Position.Fingers.Finger1 = cmds[6]/FINGER_FACTOR
            traj.Position.Fingers.Finger2 = cmds[7]/FINGER_FACTOR
            traj.Position.Fingers.Finger3 = cmds[8]/FINGER_FACTOR

        elif ("7dof" == self.arm_dof):
            traj.Position.Actuators.Actuator1 = cmds[0]
            traj.Position.Actuators.Actuator2 = cmds[1]
            traj.Position.Actuators.Actuator3 = cmds[2]
            traj.Position.Actuators.Actuator4 = cmds[3]
            traj.Position.Actuators.Actuator5 = cmds[4]
            traj.Position.Actuators.Actuator6 = cmds[5]
            traj.Position.Actuators.Actuator7 = cmds[6]
            traj.Position.HandMode = 2
            traj.Position.Fingers.Finger1 = cmds[7]/FINGER_FACTOR
            traj.Position.Fingers.Finger2 = cmds[8]/FINGER_FACTOR
            traj.Position.Fingers.Finger3 = cmds[9]/FINGER_FACTOR
            #rospy.logerr("send_angular_vel_cmds:[%f] [%f] [%f] [%f] [%f] [%f] [%f]" %(cmds[0], cmds[1], cmds[2], cmds[3], cmds[4], cmds[5], cmds[6]))

        self.SendAdvanceTrajectory(traj)
    
    def send_cartesian_vel_cmd(self):
        self.SendAdvanceTrajectory(self._cart_cmd)
    
    def get_angular_position(self):
        pos = AngularPosition()
        api_stat= self.GetAngularPosition(byref(pos))
        
        if ( NO_ERROR_KINOVA == api_stat):
            if ("6dof" == self.arm_dof):
                ret = [deg_to_rad(pos.Actuators.Actuator1),
                       deg_to_rad(pos.Actuators.Actuator2 - 180.0),
                       deg_to_rad(pos.Actuators.Actuator3 - 180.0),
                       deg_to_rad(pos.Actuators.Actuator4),
                       deg_to_rad(pos.Actuators.Actuator5),
                       deg_to_rad(pos.Actuators.Actuator6),
                       deg_to_rad(pos.Fingers.Finger1) * FINGER_FACTOR,
                       deg_to_rad(pos.Fingers.Finger2) * FINGER_FACTOR,
                       deg_to_rad(pos.Fingers.Finger3) * FINGER_FACTOR]

            elif ("7dof" == self.arm_dof):
                ret = [deg_to_rad(pos.Actuators.Actuator1),
                       deg_to_rad(pos.Actuators.Actuator2 - 180.0),
                       deg_to_rad(pos.Actuators.Actuator3),
                       deg_to_rad(pos.Actuators.Actuator4 - 180.0),
                       deg_to_rad(pos.Actuators.Actuator5),
                       deg_to_rad(pos.Actuators.Actuator6 - 180.0),
                       deg_to_rad(pos.Actuators.Actuator7),
                       deg_to_rad(pos.Fingers.Finger1) * FINGER_FACTOR,
                       deg_to_rad(pos.Fingers.Finger2) * FINGER_FACTOR,
                       deg_to_rad(pos.Fingers.Finger3) * FINGER_FACTOR]
        else:
            rospy.loginfo("Kinova API failed: GetAngularPosition (%d)",api_stat)
            ret = []

        self.handle_comm_err(api_stat)
        return ret

    def get_angular_velocity(self):
        """
        Velocity reported by API is incorrect so it needs to be corrected by a factor of 2
        """
        vel = AngularPosition()
        api_stat = self.GetAngularVelocity(byref(vel))

        if (NO_ERROR_KINOVA == api_stat):
            if ("6dof" == self.arm_dof):
                ret = [deg_to_rad(vel.Actuators.Actuator1 *2),
                       deg_to_rad(vel.Actuators.Actuator2 *2),
                       deg_to_rad(vel.Actuators.Actuator3 *2),
                       deg_to_rad(vel.Actuators.Actuator4 *2),
                       deg_to_rad(vel.Actuators.Actuator5 *2),
                       deg_to_rad(vel.Actuators.Actuator6 *2),
                       deg_to_rad(vel.Fingers.Finger1 *2) *FINGER_FACTOR,
                       deg_to_rad(vel.Fingers.Finger2 *2) *FINGER_FACTOR,
                       deg_to_rad(vel.Fingers.Finger3 *2) *FINGER_FACTOR]
            elif ("7dof" == self.arm_dof):
                ret = [deg_to_rad(vel.Actuators.Actuator1 *2),
                       deg_to_rad(vel.Actuators.Actuator2 *2),
                       deg_to_rad(vel.Actuators.Actuator3 *2),
                       deg_to_rad(vel.Actuators.Actuator4 *2),
                       deg_to_rad(vel.Actuators.Actuator5 *2),
                       deg_to_rad(vel.Actuators.Actuator6 *2),
                       deg_to_rad(vel.Actuators.Actuator7 *2),
                       deg_to_rad(vel.Fingers.Finger1 *2) *FINGER_FACTOR,
                       deg_to_rad(vel.Fingers.Finger2 *2) *FINGER_FACTOR,
                       deg_to_rad(vel.Fingers.Finger3 *2) *FINGER_FACTOR]
        else:
            rospy.loginfo("Kinova API failed: GetAngularVelocity (%d)",api_stat)
            ret = []

        self.handle_comm_err(api_stat)
        return ret
    
    def get_sensor_data(self):

        current = AngularPosition()
        api_stat = self.GetAngularCurrentMotor(byref(current))
        ret = [0]*2

        if (NO_ERROR_KINOVA == api_stat):
            if ("6dof" == self.arm_dof):
                ret[0] = [current.Actuators.Actuator1,
                          current.Actuators.Actuator2,
                          current.Actuators.Actuator3,
                          current.Actuators.Actuator4,
                          current.Actuators.Actuator5,
                          current.Actuators.Actuator6,
                          current.Fingers.Finger1,
                          current.Fingers.Finger2,
                          current.Fingers.Finger3]

            elif ("7dof" == self.arm_dof):
                ret[0] = [current.Actuators.Actuator1,
                          current.Actuators.Actuator2,
                          current.Actuators.Actuator3,
                          current.Actuators.Actuator4,
                          current.Actuators.Actuator5,
                          current.Actuators.Actuator6,
                          current.Actuators.Actuator7,
                          current.Fingers.Finger1,
                          current.Fingers.Finger2,
                          current.Fingers.Finger3]
        else:
            rospy.loginfo("Kinova API failed: GetAngularCurrentMotor (%d)",api_stat)
            ret[0] = []

        self.handle_comm_err(api_stat)
        info = SensorInfo()
        api_stat = self.GetSensorsInfo(byref(info))

        if (NO_ERROR_KINOVA == api_stat):
            if ("6dof" == self.arm_dof):
                ret[1] = [info.ActuatorTemp1,
                          info.ActuatorTemp2,
                          info.ActuatorTemp3,
                          info.ActuatorTemp4,
                          info.ActuatorTemp5,
                          info.ActuatorTemp6,
                          info.FingerTemp1,
                          info.FingerTemp2,
                          info.FingerTemp3]
            elif ("7dof" == self.arm_dof):
                ret[1] = [info.ActuatorTemp1,
                          info.ActuatorTemp2,
                          info.ActuatorTemp3,
                          info.ActuatorTemp4,
                          info.ActuatorTemp5,
                          info.ActuatorTemp6,
                          info.ActuatorTemp7,
                          info.FingerTemp1,
                          info.FingerTemp2,
                          info.FingerTemp3]
        else:
            rospy.loginfo("Kinova API failed: GetSensorsInfo (%d)",api_stat)
            ret[1] = []

        self.handle_comm_err(api_stat)
        return ret
        
    def get_angular_force(self):
        force = AngularPosition()
        api_stat = self.GetAngularForce(byref(force))

        if(NO_ERROR_KINOVA == api_stat):
            if ("6dof" == self.arm_dof):
                ret = [force.Actuators.Actuator1,
                       force.Actuators.Actuator2,
                       force.Actuators.Actuator3,
                       force.Actuators.Actuator4,
                       force.Actuators.Actuator5,
                       force.Actuators.Actuator6,
                       force.Fingers.Finger1,
                       force.Fingers.Finger2,
                       force.Fingers.Finger3]

            elif ("7dof" == self.arm_dof):
                ret = [force.Actuators.Actuator1,
                       force.Actuators.Actuator2,
                       force.Actuators.Actuator3,
                       force.Actuators.Actuator4,
                       force.Actuators.Actuator5,
                       force.Actuators.Actuator6,
                       force.Actuators.Actuator7,
                       force.Fingers.Finger1,
                       force.Fingers.Finger2,
                       force.Fingers.Finger3]

        else:
            rospy.loginfo("Kinova API failed: GetAngularForce (%d)",api_stat)
            ret = []

        self.handle_comm_err(api_stat)
        return ret

