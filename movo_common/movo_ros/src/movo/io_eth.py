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
 
 \file   udp_socket.py

 \brief  This module contains a threaded ethernet UDP communication driver

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import select
import socket
import threading
import os
import array

class IoEthThread(object):
    def __init__(self,remote_address,tx_queue,rx_queue,max_packet_size=1500):
        self.tx_queue = tx_queue
        self.rx_queue = rx_queue
        self.max_packet_size = max_packet_size
        self.remote_address = remote_address
 
        """
        Initialize the UDP connection
        """
        try:
            self.conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.conn.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.conn.setblocking(0)
            self.conn.bind(('',self.remote_address[1]))
            self.conn.connect(self.remote_address)
        except:
            try:
                self.conn.close()
            except:
                pass
            self.link_up = False
            return

        self.need_to_terminate = False
        self.listen_terminate_mutex = threading.RLock()
        self.transmit_terminate_mutex = threading.RLock()
        self.listenThread   = threading.Thread(target = self.listen)
        self.transmitThread = threading.Thread(target = self.transmit)
        self.listenThread.start()
        self.transmitThread.start()        
        self.link_up = True
    
    def Close(self):
        with self.listen_terminate_mutex, self.transmit_terminate_mutex:
            self.need_to_terminate = True
        
        assert(self.listenThread)
        assert(self.transmitThread)
        self.listenThread.join()
        self.transmitThread.join()
        self.conn.close()
        self.link_up = False
        self.tx_queue.close()
        self.rx_queue.close()
        
    def listen(self):
        while True:
            with self.listen_terminate_mutex:
                if self.need_to_terminate:
                    break
            result = select.select([self.conn],[],[],1.0)
            if (len(result[0])>0):
                message = result[0][0].recv(self.max_packet_size)
                message_bytes= array.array('B',message)
                self.rx_queue.put(message_bytes)
            
    def transmit(self):
        while True:
            with self.transmit_terminate_mutex:
                if self.need_to_terminate:
                    break    
            result = select.select([self.tx_queue._reader],[],[],1.0)
            if (len(result[0])>0):
                data = result[0][0].recv()
                self.conn.sendall(data.tostring())
        
