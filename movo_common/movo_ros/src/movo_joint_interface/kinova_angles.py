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
 
 \file   angles.py

 \brief  This module contains a collection of angle operations for
         rotational calculations

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import math

M_PI = math.pi

def deg_to_rad(degrees):
    return degrees * (M_PI / 180.0)

def rad_to_deg(rads):
    return rads * (180.0 / M_PI)
    
def wrap_angle(angle_rad):
    ret = ( angle_rad + math.pi) % (2 * math.pi ) - math.pi
    return ret

def get_smallest_difference_to_cont_angle(desired, current):

    previous_rev = math.floor(current / (2 * M_PI))
    next_rev = math.ceil(current / (2 * M_PI))
    if (math.fabs(current - previous_rev * 2 * M_PI) < math.fabs(current - next_rev * 2 * M_PI)):
        current_rev = previous_rev
    else:
        current_rev = next_rev

    lowVal = (current_rev - 1) * 2 * M_PI + desired
    medVal = current_rev * 2 * M_PI + desired
    highVal = (current_rev + 1) * 2 * M_PI + desired;
    if ((math.fabs(current - lowVal) <= math.fabs(current - medVal)) and (math.fabs(current - lowVal) <= math.fabs(current - highVal))):
        return lowVal
    if ((math.fabs(current - medVal) <= math.fabs(current - lowVal)) and (math.fabs(current - medVal) <= math.fabs(current - highVal))):
        return medVal

    return highVal


    
    
    
