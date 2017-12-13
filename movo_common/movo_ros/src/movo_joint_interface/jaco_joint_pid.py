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
-----------------------------------------------------------------------"""

import rospy
from copy import deepcopy
from helpers import FilterSignals

class JacoPID(object):
    """
    PID control class

    This class implements a simplistic PID control algorithm. When first
    instantiated all the gain variables are set to zero, so calling
    the method compute_output will just return zero.
    """

    def __init__(self, kp=0.0, ki=0.0, kd=0.0):
        # initialize gains
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._filt_error = FilterSignals(2.0,1,[0.0])

        # initialize error, results, and time descriptors
        self._prev_err = 0.0
        self._cp = 0.0
        self._ci = 0.0
        self._cd = 0.0
        self._cvff = 0.0
        self._cur_time = 0.0
        self._prev_time = 0.0

        self.initialize()

    def initialize(self):
        """
        Initialize pid controller.
        """
        # reset delta t variables
        self._cur_time = rospy.get_time()
        self._prev_time = self._cur_time
        self._filt_error.Reset([0.0])

        self._prev_err = 0.0

        # reset result variables
        self._cp = 0.0
        self._ci = 0.0
        self._cd = 0.0

    def set_kp(self, invar):
        """
        Set proportional gain.
        """
        self._kp = invar

    def set_ki(self, invar):
        """
        Set integral gain.
        """
        self._ki = invar

    def set_kd(self, invar):
        """
        Set derivative gain.
        """
        self._kd = invar

    def set_vlim(self, invar):
        """
        Set derivative gain.
        """
        self._vlim = invar

    def compute_output(self, error):
        """
        Performs a PID computation and returns a control value based on
        the elapsed time (dt) and the error signal from a summing junction
        (the error parameter).
        """
        err = self._filt_error.Update([error])[0]
        self._cur_time = rospy.get_time()  # get t
        dt = self._cur_time - self._prev_time  # get delta t
        de = err - self._prev_err  # get delta error
        
            
        self._cp = err  # proportional term
        self._ci += err * dt  # integral term

        self._cd = 0
        if dt > 0:  # no div by zero
            self._cd = de / dt  # derivative term

        self._prev_time = self._cur_time  # save t for next pass
        self._prev_err = err  # save t-1 error
        
        output = ((self._kp * self._cp) + (self._ki * self._ci) +
                  (self._kd * self._cd))

        # sum the terms and return the result
        return output
