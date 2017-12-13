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
----------------------------------------------------------------------"""

from movo.io_eth import IoEthThread
from gripper_io import GripperIO
from modbus_crc import verify_modbus_rtu_crc
from movo.crc32 import calc_crc32, valid_crc32
from copy import deepcopy
import array
import rospy
import multiprocessing
import threading
import select

R85_PACKET_SIZE_BYTES = 72

def add_bytes(list_to_append,var2,bits):
    if bits % 2:
        return False
    
    bytes_to_make = bits/8
    tmp_list = []
    for i in range(0,bytes_to_make):
        shift = bits - 8*(i+1)
        tmp_list.insert(0,(var2 >> shift) & 0xFF)    
    
    for tmp in tmp_list:
        list_to_append.append(tmp)

class Robotiq85Gripper:
    def __init__(self,num_grippers=1,movo_ip='10.66.171.5',rcv_timeout=0.1):

        self._shutdown_driver=False
        self._rcv_timeout = rcv_timeout        
        """
        Create the thread to run MOVO Linear actuator command interface
        """
        self.tx_queue_ = multiprocessing.Queue()
        self.rx_queue_ = multiprocessing.Queue()
        self.comm = IoEthThread((movo_ip,6238),
                                self.tx_queue_,
                                self.rx_queue_,
                                max_packet_size=R85_PACKET_SIZE_BYTES)
        
        self._gripper = []
        self._num_grippers = num_grippers
        for i in range(self._num_grippers):
            self._gripper.append(GripperIO(i))
        
        self.init_success = True

    def shutdown(self):
        self._shutdown_driver = True
        rospy.loginfo("Movo R85 gripper has called the Shutdown method, terminating")
        self.comm.Close()
        self.tx_queue_.close()
        self.rx_queue_.close()
    
    def _transact(self,cmd,rx_bytes=0):
        
        """
        Create a byte array with the CRC from the command
        """
        
        cmd_bytes = [0]*68
        cmd_bytes[0] = 0
        cmd_bytes[1] = 0
        cmd_bytes[2] = (len(cmd)&0xFF)
        cmd_bytes[3] = (rx_bytes&0xFF)
        
        j=4
        for b in cmd:
            cmd_bytes[j] = b
            j+=1
        cmd_bytes = array.array('B',cmd_bytes)
        
        """
        Generate the CRC for the command bytes
        """
        crc = calc_crc32(cmd_bytes)
        add_bytes(cmd_bytes,crc,32)
        
        """
        Send it
        """
        self.tx_queue_.put(cmd_bytes) 
        rsp_data = None
        try:
            result = select.select([self.rx_queue_._reader],[],[],self._rcv_timeout)
        
            if len(result[0]) > 0:
                rsp = result[0][0].recv()
                if (valid_crc32(rsp)) and (len(rsp) == 72):
                    rsp_data  = rsp[4:rx_bytes+4]
        except:
            pass
        
        return rsp_data
    
    def process_act_cmd(self,dev=0):
        if (dev >= self._num_grippers) or (self._shutdown_driver):
            return False
        rsp = self._transact(self._gripper[dev].act_cmd,8)
        
        if (None == rsp):
            return False
        return verify_modbus_rtu_crc(rsp)
        
    def process_stat_cmd(self,dev=0):
        if (dev >= self._num_grippers) or (self._shutdown_driver):
            return False
        rsp = self._transact(self._gripper[dev].stat_cmd,21)
        if (None == rsp):
            return False
        return self._gripper[dev].parse_rsp(rsp)

    def activate_gripper(self,dev=0):
        if (dev >= self._num_grippers):
            return
        self._gripper[dev].activate_gripper()
    
    def deactivate_gripper(self,dev=0):
        if (dev >= self._num_grippers):
            return
        self._gripper[dev].deactivate_gripper()
        
    def activate_emergency_release(self,dev=0,open_gripper=True):
        if (dev >= self._num_grippers):
            return
        self._gripper[dev].activate_emergency_release(open_gripper)
                
    def deactivate_emergency_release(self,dev=0):
        if (dev >= self._num_grippers):
            return
        self._gripper[dev].deactivate_emergency_release()

    def goto(self, dev=0, pos=0.0, vel=1.0, force=1.0):
        if (dev >= self._num_grippers):
            return
        self._gripper[dev].goto(pos, vel, force)

    def stop(self,dev=0):
        if (dev >= self._num_grippers):
            return
        self._gripper[dev].stop()
                    
    def is_ready(self,dev=0):
        if (dev >= self._num_grippers):
            return False
        return self._gripper[dev].is_ready()

    def is_reset(self,dev=0):
        if (dev >= self._num_grippers):
            return False
        return self._gripper[dev].is_reset()

    def is_moving(self,dev=0):
        if (dev >= self._num_grippers):
            return False
        return self._gripper[dev].is_moving()

    def is_stopped(self,dev=0):
        if (dev >= self._num_grippers):
            return False
        return self._gripper[dev].is_moving()

    def object_detected(self,dev=0):
        if (dev >= self._num_grippers):
            return False
        return self._gripper[dev].object_detected()

    def get_fault_status(self,dev=0):
        if (dev >= self._num_grippers):
            return 0
        return self._gripper[dev].get_fault_status()

    def get_pos(self,dev=0):
        if (dev >= self._num_grippers):
            return 0
        return self._gripper[dev].get_pos()

    def get_req_pos(self,dev=0):
        if (dev >= self._num_grippers):
            return 0
        return self._gripper[dev].get_req_pos()

    def get_current(self,dev=0):
        if (dev >= self._num_grippers):
            return 0
        return self._gripper[dev].get_current()
