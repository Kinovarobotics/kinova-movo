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
from movo_msgs.msg import PanTiltCmd, PanTiltFdbk, PVA
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.srv import QueryTrajectoryState,QueryTrajectoryStateResponse
from io_eth import IoEthThread
import multiprocessing
import threading
import select
import rospy
import operator

class PanTiltIO(object):
    def __init__(self,movo_ip='10.66.171.5'):        
        self.init_success = False
        
        """
        Create the thread to run MOVO Linear actuator command interface
        """
        self._cmd_buffer = multiprocessing.Queue()
        self.tx_queue_ = multiprocessing.Queue()
        self.rx_queue_ = multiprocessing.Queue()
        self.comm = IoEthThread((movo_ip,6237),
                                self.tx_queue_,
                                self.rx_queue_,
                                max_packet_size=KINOVA_ACTUATOR_RSP_SIZE_BYTES)
                                    
        
        if (False == self.comm.link_up):
            rospy.logerr("Could not open socket for MOVO pan_tilt...exiting")
            self.Shutdown()
            return

        """
        Initialize the publishers and subscribers for the node
        """
        self.cmd_data = PanTiltCmd()
        self._last_cmd = JointTrajectoryPoint()
        self._init_cmds = True
        self.s = rospy.Subscriber("/movo/head/cmd", PanTiltCmd, self._add_motion_command_to_queue)
        self._jcc = rospy.Subscriber("/movo/head_controller/command",JointTrajectory,self._add_traj_command_to_queue)
        
        self.actuator_data = PanTiltFdbk()
        self.actuator_pub = rospy.Publisher("/movo/head/data", PanTiltFdbk, queue_size=10)
        self.js = JointState()
        self.js_pub = rospy.Publisher("/movo/head/joint_states", JointState, queue_size=10)
        self._jcs = JointTrajectoryControllerState()
        self._jcs_pub = rospy.Publisher("/movo/head_controller/state",JointTrajectoryControllerState,queue_size=10)
        
        self._jc_srv = rospy.Service('/movo/head_controller/query_state', QueryTrajectoryState, self._handle_state_query)

        """
        Start the receive handler thread
        """
        self.need_to_terminate = False
        self.terminate_mutex = threading.RLock()
        self.last_rsp_rcvd = rospy.get_time()
        self._rcv_thread   = threading.Thread(target = self._run)
        self._rcv_thread.start()
        
        self._t1 = rospy.Timer(rospy.Duration(0.01),self._update_command_queue)
        
        """
        Start streaming continuous data
        """
        rospy.loginfo("Stopping the data stream")
        if (False == self._continuous_data(False)):
            rospy.logerr("Could not stop MOVO pan_tilt communication stream")
            self.Shutdown()
            return

        """
        Start streaming continuous data
        """ 
        rospy.loginfo("Starting the data stream")
        if (False == self._continuous_data(True)):
            rospy.logerr("Could not start MOVO pan_tilt communication stream")
            self.Shutdown()
            return

        rospy.loginfo("Movo Pan-Tilt Head Driver is up and running")
        self.init_success = True
        
    def Shutdown(self):
        with self.terminate_mutex:
            self.need_to_terminate = True
        rospy.loginfo("Movo pan_tilt has called the Shutdown method, terminating")
        self.js_pub.unregister()
        self.actuator_pub.unregister()
        self.s.unregister()
        self._jcs_pub.unregister()
        self._jcc.unregister()
        self._jc_srv.shutdown()
        self.comm.Close()
        self.tx_queue_.close()
        self.rx_queue_.close()    
    
    def _run(self):
        
        while not self.need_to_terminate:
            """
            Run until signaled to stop
            Perform the actions defined based on the flags passed out
            """
            result = select.select([self.rx_queue_._reader],[],[],0.02)
            if len(result[0]) > 0:
                data = result[0][0].recv()
                with self.terminate_mutex:
                    if not self.need_to_terminate:
                        self._handle_rsp(data)
                        
    def _handle_state_query(self,req):
        tmp = QueryTrajectoryStateResponse()
        tmp.name = ['pan_joint','tilt_joint']
        tmp.position = [self.actuator_data.pan.pos_rad,self.actuator_data.tilt.pos_rad]
        tmp.velocity = [self.actuator_data.pan.vel_rps,self.actuator_data.tilt.vel_rps]
        tmp.acceleration = [0.0,0.0]
        return tmp
    
    def _update_command_queue(self,event):
        """
        Send it
        """
        if not self._cmd_buffer.empty():
            tmp = self._cmd_buffer.get_nowait()
            
            cmds = [KINOVA_ACTUATOR_CMD_ID,[convert_float_to_u32(tmp.positions[0]),
                                            convert_float_to_u32(tmp.velocities[0]),
                                            convert_float_to_u32(tmp.accelerations[0]),
                                            convert_float_to_u32(tmp.positions[1]),
                                            convert_float_to_u32(tmp.velocities[1]),
                                            convert_float_to_u32(tmp.accelerations[1])]]
            
            self._last_cmd = tmp
            
            cmd_bytes = generate_cmd_bytes(cmds)
            self.tx_queue_.put(cmd_bytes)
        
    def _add_motion_command_to_queue(self,command):
        tmp = JointTrajectory()
        tmp.header = command.header
        tmp.joint_names = ['pan_joint','pan_joint']
        tmp.points = [JointTrajectoryPoint()]
        tmp.points[0].positions = [command.pan_cmd.pos_rad,command.tilt_cmd.pos_rad]
        tmp.points[0].velocities = [command.pan_cmd.vel_rps,command.tilt_cmd.vel_rps]
        tmp.points[0].accelerations = [command.pan_cmd.acc_rps2,command.tilt_cmd.acc_rps2]
        
        self._cmd_buffer.put(tmp.points[0])
        
    def _add_traj_command_to_queue(self,msg):
        for pnt in msg.points:
            self._cmd_buffer.put(pnt)
            
    def _add_config_command_to_queue(self,gp_cmd, gp_param):
        try:
            cmds = [EIB_GENERAL_CMD_ID,[gp_cmd,gp_param]]
            cmd_bytes = generate_cmd_bytes(cmds)
            self.tx_queue_.put(cmd_bytes)
        except:
            rospy.logerr("Config param failed, it is probably not known")
            return

    def _handle_rsp(self,data_bytes):
        valid_data = validate_response(data_bytes,KINOVA_ACTUATOR_RSP_SIZE_BYTES)
        if (False == valid_data):
            self.last_rsp_rcvd = rospy.get_time()
            rospy.logerr("bad movo pan_tilt data packet")
            return
        
        self.last_rsp_rcvd = rospy.get_time()
        
        data_bytes = data_bytes[2:]

        rsp_data  = array.array('I',data_bytes.tostring()).tolist()       
        rsp_data = rsp_data[:(len(rsp_data)-1)]
        
        rsp = [convert_u32_to_float(i) for i in rsp_data]
        
        self.actuator_data.header.stamp = rospy.get_rostime()
        self.actuator_data.header.seq +=1
        for i in range(2):
            if (i==0):
                self.actuator_data.pan.current =  (rsp[10*i])
                self.actuator_data.pan.pos_rad =  (rsp[10*i+1])
                self.actuator_data.pan.vel_rps =  (rsp[10*i+2])
                self.actuator_data.pan.torque_nm =  (rsp[10*i+3])
                self.actuator_data.pan.pwm =  (rsp[10*i+4])
                self.actuator_data.pan.encoder_rad =  (rsp[10*i+5])
                self.actuator_data.pan.accel.x =  (rsp[10*i+6])
                self.actuator_data.pan.accel.y =  (rsp[10*i+7])
                self.actuator_data.pan.accel.z =  (rsp[10*i+8])
                self.actuator_data.pan.temperature_degC =  (rsp[10*i+9])
            else:
                self.actuator_data.tilt.current =  (rsp[10*i])
                self.actuator_data.tilt.pos_rad =  (rsp[10*i+1])
                self.actuator_data.tilt.vel_rps =  (rsp[10*i+2])
                self.actuator_data.tilt.torque_nm =  (rsp[10*i+3])
                self.actuator_data.tilt.pwm =  (rsp[10*i+4])
                self.actuator_data.tilt.encoder_rad =  (rsp[10*i+5])
                self.actuator_data.tilt.accel.x =  (rsp[10*i+6])
                self.actuator_data.tilt.accel.y =  (rsp[10*i+7])
                self.actuator_data.tilt.accel.z =  (rsp[10*i+8])
                self.actuator_data.tilt.temperature_degC =  (rsp[10*i+9])

        if not rospy.is_shutdown():
            self.actuator_pub.publish(self.actuator_data)
            
            self.js.header.stamp = self.actuator_data.header.stamp
            self.js.name = ['pan_joint','tilt_joint']
            self.js.position = [self.actuator_data.pan.pos_rad,self.actuator_data.tilt.pos_rad]
            self.js.velocity = [self.actuator_data.pan.vel_rps,self.actuator_data.tilt.vel_rps]
            self.js.effort = [self.actuator_data.pan.torque_nm,self.actuator_data.tilt.torque_nm]
            
            self.js_pub.publish(self.js)
            
            self._jcs.header.stamp = self.actuator_data.header.stamp
            self._jcs.joint_names = ['pan_joint','tilt_joint']
            self._jcs.desired = self._last_cmd
            self._jcs.actual.positions = self.js.position
            self._jcs.actual.velocities = self.js.velocity
            self._jcs.actual.accelerations = [0.0,0.0]
            try:
                self._jcs.error.positions = map(operator.sub,self._jcs.desired.positions,self._jcs.actual.positions)
                self._jcs.error.velocities = map(operator.sub,self._jcs.desired.velocities,self._jcs.actual.velocities)
            except:
                self._jcs.error.positions = [0.0]*2
                self._jcs.error.velocities = [0.0]*2
                self._jcs.desired = self._jcs.actual
                self._last_cmd = self._jcs.actual 
                
            self._jcs.error.accelerations = [0.0,0.0]
            self._jcs_pub.publish(self._jcs)
            
        rospy.logdebug("feedback received from movo")
        
    def _continuous_data(self,start_cont):
        ret = False
        
        if (True == start_cont):
            r = rospy.Rate(10)
            start_time = rospy.get_time()
            while ((rospy.get_time() - start_time) < 3.0) and (False == ret):
                self._add_config_command_to_queue(0,1)
                r.sleep()
                if  ((rospy.get_time() - self.last_rsp_rcvd) < 0.05):
                    ret = True
        else:
            start_time = rospy.get_time()
            while ((rospy.get_time() - start_time) < 3.0) and (False == ret):
                self._add_config_command_to_queue(0,0)
                rospy.sleep(0.6)
                if  ((rospy.get_time() - self.last_rsp_rcvd) > 0.5):
                    ret = True

        return ret
