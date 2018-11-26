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
 
 \file   utils.py

 \brief  This module contains general utility functions

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import struct
import socket
import math
from crc32 import calc_crc32, valid_crc32
import array
from system_defines import *
import timeit

"""
slew limit funtion to limit the maximum rate of change
"""
def slew_limit(signal_in,signal_out,max_rate,dt):
    if (0 == dt):
        return
    requested_rate = (signal_in - signal_out)/dt
                
    if (requested_rate > max_rate): 
        signal_out += max_rate * dt
    elif (requested_rate >= -max_rate): 
        signal_out = signal_in
    else: 
        signal_out += -max_rate * dt
    
    return signal_out


"""
Make a 16-bit value from two 8-bit values
"""
def m16(byte_array):

    return (( byte_array[0] << 8) & 0xFF00) | (byte_array[1] & 0x00FF)

"""
Make a 32-bit value from four 8-bit values
"""
def m32(byte_array):
    ret = 0;
    ret |= (byte_array[0] & 0xFF) << 24
    ret |= (byte_array[1] & 0xFF) << 16
    ret |= (byte_array[2] & 0xFF) << 8
    ret |= (byte_array[3] & 0xFF)

    return ret

def generate_cmd_bytes(cmd):
    cmd_bytes = []
    
    add_bytes(cmd_bytes,cmd[0],16)
    for cmd_var in cmd[1]:
        add_bytes(cmd_bytes,cmd_var,32)
        
    cmd_bytes = array.array('B',cmd_bytes)
    
    """
    Generate the CRC for the command bytes
    """
    crc = calc_crc32(cmd_bytes)
    add_bytes(cmd_bytes,crc,32)
   
    return cmd_bytes
    
def validate_response(rsp,expected_len):
    
    """
    Check the CRC and 
    """
    if (valid_crc32(rsp)) and (len(rsp) == expected_len):
        return True
       
    """
    Not a valid CRC
    """
    return False
    
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

"""
For IEEE754 processors this function converts a 32-bit floating point number to
a 32-bit integer representation
"""
def convert_float_to_u32(value):
    return struct.unpack('=I', struct.pack('=f', value))[0]

"""
For IEEE754 processors this function converts a 32-bit integer representation
of a floating point value to float representation
"""
def convert_u32_to_float(bits):
    return struct.unpack('=f', struct.pack('=I', bits))[0]

def convert_u64_to_double(high_word,low_word):
    temp = (high_word << 32) & 0xFFFFFFFF00000000
    temp |= (low_word & 0x00000000FFFFFFFF)
    
    return struct.unpack('=d', struct.pack('=Q', temp))[0]

"""
Used to convert a byte array (string) into an array of 32-bit values
"""
def convert_byte_data_to_U32(data):

    rx_dat = [];
    k = 0;
    
    #
    # Convert the string into a byte array
    #
    
    for x in range(0,len(data)):
        rx_dat.append(ord(data[x]));
        
    number_of_u32s = (len(rx_dat)/4)

    #
    # Convert the byte array into an array of 32bit values
    #
    converted = [0]*number_of_u32s;
    for x in range(0,number_of_u32s):
        converted[x] = int((((rx_dat[k]   << 24) & 0xFF000000)) |
                        (((rx_dat[k+1] << 16) & 0x00FF0000)) |
                        (((rx_dat[k+2] << 8)  & 0x0000FF00)) |
                          (rx_dat[k+3] & 0x000000FF));

        k+=4;
        
    return converted;

"""
Used to convert an IP address string in dotted quad format to an integer
"""  
def dottedQuadToNum(ip):
    "convert decimal dotted quad string to long integer"
    return struct.unpack('I',socket.inet_aton(ip))[0]

"""
Used to convert an IP address in integer format to a dotted quad string
""" 
def numToDottedQuad(n):
    "convert long int to dotted quad string"
    return socket.inet_ntoa(struct.pack('I',n))

def limit_f(signal_in, limit):
    
    if (signal_in > abs(limit)):
        return abs(limit)
    elif (signal_in <= -abs(limit)):
        return -abs(limit)
    else:
        return signal_in
    
def clamp_value_f(value,lower_limit,upper_limit):
    
    if (value < lower_limit):
        value = lower_limit;
    elif (value > upper_limit):
        value = upper_limit;
    
    return value;

def minimum_f(input1,input2):
    
    if (math.fabs(input1) > math.fabs(input2)):
        return input2
    return input1

def maximum_f(input1,input2):
    
    if (math.fabs(input1) < math.fabs(input2)):
        return input2
    return input1

def approx_equal(in_1,in_2,max_delta):
    
    if abs(in_1 - in_2) <= max_delta :
        return True
    return False




