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
--------------------------------------------------------------------"""

import rospy
import math
import threading
from copy import deepcopy

import socket
import fcntl
import struct

"""
Used to convert an IP address string in dotted quad format to an integer
"""  
def dottedQuadToNum(ip):
    "convert decimal dotted quad string to long integer"
    return struct.unpack('I',socket.inet_aton(ip))[0]     


def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])
    

def limit(signal_in,signal_limit):

    if (signal_in >= signal_limit):
        ret = signal_limit 
    elif (signal_in <= -signal_limit):
        ret = -signal_limit
    else:
        ret = signal_in
    return ret

                
class RateLimitSignals(object):
    def __init__(self,max_rate,num_sigs=1,sig_init=[0.0]):
        self._max_rate = max_rate
        self._sigout = deepcopy(sig_init)
        self._sigin  = deepcopy(sig_init)
        self._numsigs = num_sigs
        self._prev_time = rospy.get_time()
    
    def SetMaxRate(self,rate):
        self._max_rate = rate
    def Reset(self,sig_init=[0.0]):
        self._sigout = deepcopy(sig_init)
        self._sigin  = deepcopy(sig_init)
        self._prev_time = rospy.get_time()
    def Update(self,sigin):
        self._sigin  = deepcopy(sigin)
        now = rospy.get_time()
        dt = now - self._prev_time
        self._prev_time = now
        if (dt > 0.0):
            for i in range(self._numsigs):
                requested_rate = (self._sigin[i] - self._sigout[i])/dt
                            
                if (requested_rate > self._max_rate[i]):
                    self._sigout[i] += self._max_rate[i] * dt
                elif (requested_rate < -self._max_rate[i]):
                    self._sigout[i] += -self._max_rate[i] * dt
                else:
                    self._sigout[i] = sigin[i]
        ret = deepcopy(self._sigout)
        
        return ret

class DifferentiateSignals(object):
    def __init__(self,num_sigs=1,sig_init=[0.0]):
        self._prev_sigin = deepcopy(sig_init)
        self._sigin = deepcopy(sig_init)
        self._numsigs = num_sigs
        self._prev_time = rospy.get_time()
    def Reset(self,sig_init=[0.0]):
        self._prev_sigin = deepcopy(sig_init)
        self._sigin = deepcopy(sig_init)
        self._prev_time = rospy.get_time()
    def Update(self,sigin):
        self._sigin = deepcopy(sigin)
        now = rospy.get_time()
        dt = now - self._prev_time
        self._prev_time = now
        ret = [0.0]*self._numsigs
        if (dt > 0.0):
            ret = [(self._sigin[i]-self._prev_sigin[i])/dt for i in range(self._numsigs)]
        self._prev_sigin = deepcopy(self._sigin)

        return ret
      

class FilterSignals(object):
    def __init__(self,cutoff_freq,num_sigs=1,sig_init=[0.0]):
        self._cutoff_freq = cutoff_freq
        self._sigout = deepcopy(sig_init)
        self._sigin = deepcopy(sig_init)
        self._numsigs = num_sigs
        self._prev_time = rospy.get_time()
        
    def SetCuttoffFreq(self,cutoff_freq):
        self._cutoff_freq = cutoff_freq
    def Reset(self,sig_init=[0.0]):
        self._sigout = deepcopy(sig_init)
        self._sigin = deepcopy(sig_init)
        self._prev_time = rospy.get_time()
    def Update(self,sigin):
        self._sigin = deepcopy(sigin)
        now = rospy.get_time()
        dt = now - self._prev_time
        self._prev_time = now
        if (dt > 0.0):
            filter_const = math.exp((-1.0 * (self._cutoff_freq * (2.0 * math.pi))) * dt)
            self._sigout = [self._sigin[i] + (filter_const * (self._sigout[i] - self._sigin[i])) for i in range(self._numsigs)]
        ret = deepcopy(self._sigout)
        
        return ret


