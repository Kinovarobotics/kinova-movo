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

 \file   faultlog_parser.py

 \brief  This module contains a class for parsing the faultlog into
         human readable form

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from system_defines import *
from utils          import *
import webbrowser
import os
import time
import rospkg
import shutil

"""
Class for faultlog extraction
"""
class FaultlogParser:
    def __init__(self,faultlog_array):
               
        """
        If this path does not exists make it it is the default path for the
        faultlog extraction
        """
        if (False == os.path.exists(os.getcwd()+"/SI_FAULTLOGS/img")):
            os.makedirs(os.getcwd()+"/SI_FAULTLOGS/img")
        
        """
        Get the directory and make sure the user did not cancel
        """
        self.dir_path = os.getcwd()+ "/SI_FAULTLOGS"
        
        """
        Copy the logo over to the directory
        """
        rospack = rospkg.RosPack()
        img = rospack.get_path('movo_ros') + "/src/movo/"
        shutil.copyfile(img+'logo.png',os.getcwd()+"/SI_FAULTLOGS/img/logo.png") 
        
        """
        Parse the faultlog array
        """
        filename = self.dir_path + "/" + "SI_FAULTOG_" + time.strftime("%m%d%Y_%H%M%S") + ".html"     
        Create_Log_File(filename,faultlog_array.data)
        try:
            webbrowser.open(filename)
        except:
            webbrowser.open(filename)        

"""
Begin faultlog file creation functions and variables
"""
MAX_FAULT_ENTRIES = 20;
NUMBER_OF_ITEMS_PER_ENTRY = 15;
NUMBER_OF_FAULT_GROUPS = 8;

fault_group_names = ["Transient Faults",
                     "Critical Faults",
                     "Communication Faults",
                     "Sensor Faults",
                     "IMU Faults",
                     "Motordrive Faults",
                     "Architecture Faults",  
                     "Internal Faults"];
                     
decode_list = [transient_fault_decode,
               critical_fault_decode,
               comm_fault_decode,
               sensor_fault_decode,
               imu_fault_decode,
               md_fault_decode,
               arch_fault_decode,
               internal_fault_decode]

"""
Define calender constants
"""
NUMBER_OF_MONTHS_PER_YEAR  = (12)
SECONDS_PER_MINUTE         = (60)
SECONDS_PER_HOUR           = (3600)
SECONDS_PER_DAY            = (86400)
SECONDS_PER_YEAR           = (31536000)
SECONDS_PER_LEAP_YEAR      = (31622400)

days_per_month = [31, #January
                  28, #February (in non-leap years)
                  31, #March
                  30, #April
                  31, #May
                  30, #June
                  31, #July
                  31, #August
                  30, #September
                  31, #October
                  30, #November
                  31  #December
                  ];
    
"""
Convert the seconds in the fault log to an actual date based on the
origin date
"""
def seconds_to_date(seconds):
    year = 2011    
    
    get_years = True;
    while (get_years):
        if (0 == year % 4):
            if (seconds - SECONDS_PER_LEAP_YEAR) >= 0:
                year +=1;
                seconds = seconds - SECONDS_PER_LEAP_YEAR 
            else:
                get_years = False;
        else:
            if (seconds - SECONDS_PER_YEAR) >= 0:
                year +=1;
                seconds = seconds - SECONDS_PER_YEAR 
            else:
                get_years = False;        
        
        
    get_month = True;
    month = 1;
    while (get_month):
        seconds_in_month = days_per_month[month - 1] * SECONDS_PER_DAY;
        if ((0 == (year % 4)) and (2 == month)):
            seconds_in_month += SECONDS_PER_DAY;
             
        if ((seconds - seconds_in_month) >= 0):
            seconds = seconds - seconds_in_month;
            month += 1;
        else:
            get_month = False;
    
    get_day = True;
    day = 1;
    while (get_day):
        if (seconds - SECONDS_PER_DAY) >= 0:
            seconds = seconds - SECONDS_PER_DAY;
            day += 1;
        else:
            get_day = False;
    
    get_hour = True;
    hour = 0;
    while(get_hour):
        if (seconds - SECONDS_PER_HOUR) >= 0:
            hour += 1;
            seconds = seconds - SECONDS_PER_HOUR;
        else:
            get_hour = False;
    
    get_minute = True;
    minute = 0;
    while(get_minute):
        if (seconds - SECONDS_PER_MINUTE) >= 0:
            minute += 1;
            seconds = seconds - SECONDS_PER_MINUTE;
        else:
            get_minute = False;
            
    sec = seconds;
    return "%(1)02d-%(2)02d-%(3)02d  %(4)02d:%(5)02d:%(6)02d (EST)"  %{"1":month,"2":day,"3":year,"4":hour,"5":minute,"6":sec}
 
"""
Define some helper functions for creating HTML
"""
def trMsgHex( a, v, html):
    html.append("<tr><td style=\"text-align:right;font-weight:bold;\">%(1)s</td><td>x%(2)08X</td></tr>" %{"1":a, "2":v});
def trMsgLongHex( a, v1, v2, html):
    html.append("<tr><td style=\"text-align:right;font-weight:bold;\">%(1)s</td><td>x%(2)08X%(3)08X</td></tr>" %{"1":a, "2":v1, "3":v2});
def trMsgDec( a, v, html):
    html.append("<tr><td style=\"text-align:right;font-weight:bold;\">%(1)s</td><td>%(2)d</td></tr>" %{"1":a, "2":v});
def trMsgString( a, v, html):
    html.append("<tr><td style=\"text-align:right;font-weight:bold;\">%(1)s</td><td>%(2)s</td></tr>" %{"1":a, "2":v});    
def secondsToTimeString(seconds): 
    hours = seconds / 60 / 60;
    minutes = ((seconds / 60) % 60);
    secs = seconds % 60;

    result = "%(1)d:%(2)02d:%(3)02d" %{"1":hours,"2":minutes,"3":secs};
    
    return result

"""
Helper function to decode the faults
"""
def decode_faults(value,table,html):
    for x in range(0,32):
        temp = int(math.pow(2,x))
        temp = (value & temp);
        if (temp):
            try:
                html.append("<tr><td></td><td>(x%(1)08X) %(2)s</td></tr>" %{"1":temp,"2":table[temp]});
            except:
                html.append("<tr><td></td><td>(x%(1)08X) NO_FAULT_INDICATION</td></tr>"%{"1":temp})
                
            
"""
This function creates the entire parsed faultlog file
"""            
def Create_Log_File(filename, data):

    outfile = open(filename, "w"); 
    
    """
    Create an array for holding the file lines
    """
    html = [];
    
    """
    Create the file header
    """
    html.append(("<html><head><title>SI Faultlog %s</title>\n"  %filename));
    html.append("\
    <style>\
    BODY, tr, td {\
      font-family: arial, sans;\
      font-size:11px;\
      background-color:#ffffff; \
      margin: 0px 5px 5px 30px;\
    }\
    td {\
      padding-right: 15px;\
    }\
    h1,h2,h3 {\
      font-family: tahoma;\
      color: #99cc33;\
      margin: 0px;\
    }\
    h2 {\
      font-size: 14px;\
    }\
    </style>");
    html.append("</head><body>\n")
    html.append("<div class=\"header\">\n<img src=\"img/logo.png\" alt=\"logo\" width=\"500\"/>\n</div>\n")
    
    """
    Create the faultlog instance header table
    """
    html.append("<table>\n");
    trMsgString("Filename", filename, html);
    trMsgHex("Log Version", data[0],html);
    trMsgDec("Log Size Bytes", data[1],html);
    trMsgDec("Number of Entries", data[2],html);
    trMsgDec("Latest Entry", data[3],html);
    trMsgLongHex("Serial Number",data[4], data[5],html);
    trMsgDec("SIC SW Build ID", data[6],html);
    trMsgHex("SIC SW Build Stamp", data[7],html);
    
    acc_time = secondsToTimeString(data[8]);
    trMsgString("Accumulated Time", acc_time,html);
    trMsgDec("Odometer (m)", data[9],html);
    trMsgDec("Power Cycles", data[10],html);
    html.append("</table>\n");
    
    html.append("<p>Faults are listed in the order they appear in the fault\n\
                log, not in the order in which they have occurred.</p>\n");
    
    
    """
    Append the entire raw faultlog
    """
    html.append("<!--  Raw Data From Log\n");
    raw_shorts = [0] * ((len(data) - 1) * 2);
    for i in range(0,(len(data) - 1)):
        raw_shorts[i * 2]     = (data[i] & 0xFFFF0000)>>16;
        raw_shorts[i * 2 + 1] = (data[i] & 0x0000FFFF);
        
    
    for i in range(0,len(raw_shorts)):    
        if ((i % 16) == 0):
            html.append("\n%03X: " %(i * 2));
            
        html.append("%04X " %raw_shorts[i]);
    
    html.append("\n-->");
    
    """
    Now start decoding each of the entries
    """
    fault_entries = [0] * MAX_FAULT_ENTRIES;
    
    for i in range(0,MAX_FAULT_ENTRIES):
        temp = [0] * NUMBER_OF_ITEMS_PER_ENTRY;
        for j in range(0,NUMBER_OF_ITEMS_PER_ENTRY):
            temp[j]= data[i * NUMBER_OF_ITEMS_PER_ENTRY + j + 11]
            
        fault_entries[i] = temp;
    
    for i in range(0,MAX_FAULT_ENTRIES):
        if ((i == data[3]) and (data[2])):
            html.append(("<h2 style='color: #99cc33'>Fault[ %d] (Latest Entry)</h2>\n" %i));
        else:
            html.append(("<h2>Fault[ %d]</h2>\n" %i));
        
        """
        Check if it is empty
        """
        empty = True;
        for j in range(0,NUMBER_OF_ITEMS_PER_ENTRY):
            if (0 != fault_entries[i][j]):
                empty = False; 
    
        if (empty):
            html.append("<i>empty</i>\n");
        else:
            """
            Create the faultlog entry table
            """
            html.append("<table>\n");
    
            timestamp = seconds_to_date(fault_entries[i][0]);
            trMsgString("Time Stamp", timestamp,html);
    
            runtimestamp = secondsToTimeString(fault_entries[i][1]);
            trMsgString("Runtime Stamp", runtimestamp,html);
            trMsgDec("Power Cycle", fault_entries[i][2],html);
    
            """
            Decode all the faults present. If we are in the MCU specific faults
            check the gpdata and decode that as well
            """
            for k in range(0,NUMBER_OF_FAULT_GROUPS):
                trMsgHex(fault_group_names[k], fault_entries[i][3+k],html);
                decode_faults(fault_entries[i][3+k], decode_list[k],html);
    
            """
            Represent the gpdata in floating point by default
            """
            float_rep = convert_u32_to_float(fault_entries[i][11])
            html.append("<tr><td style=\"text-align:right;font-weight:bold;\">%(1)s</td><td>x%(2)08X %(3)f</td></tr>" 
                    % {"1":"Data[0]","2":fault_entries[i][11],"3":float_rep}); 
    
            float_rep = convert_u32_to_float(fault_entries[i][12])
            html.append("<tr><td style=\"text-align:right;font-weight:bold;\">%(1)s</td><td>x%(2)08X %(3)f</td></tr>" 
                    % {"1":"Data[1]","2":fault_entries[i][12],"3":float_rep}); 
                    
                    
            """
            Finished with this entry
            """
            html.append("<table>\n");
    
    """
    Finished with this log
    """
    html.append("</body></html>\n");
       
    """
    Write the outfile and close it
    """
    output = ''.join(html);
    outfile.write(output);
    outfile.close();
    
"""
Decode a FSW
"""    
def decode_fsw(fsw_array):

    """
    Parse the array into specific faultgroups
    """
    faultGroup = [0] * NUM_OF_FAULTGROUPS;
    
    faultGroup[FAULTGROUP_TRANSIENT]     = 0
    faultGroup[FAULTGROUP_CRITICAL]      =  (fsw_array[FSW_CRITICAL_FAULTS_INDEX] & FSW_CRITICAL_FAULTS_MASK) >> FSW_CRITICAL_FAULTS_SHIFT;
    faultGroup[FAULTGROUP_COMM]          =  (fsw_array[FSW_COMM_FAULTS_INDEX] & FSW_COMM_FAULTS_MASK) >> FSW_COMM_FAULTS_SHIFT;
    faultGroup[FAULTGROUP_SENSORS]       =  (fsw_array[FSW_SENSORS_FAULTS_INDEX] & FSW_SENSORS_FAULTS_MASK) >> FSW_SENSORS_FAULTS_SHIFT;    
    faultGroup[FAULTGROUP_IMU]           =  (fsw_array[FSW_IMU_FAULTS_INDEX] & FSW_IMU_FAULTS_MASK) >> FSW_IMU_FAULTS_SHIFT;
    faultGroup[FAULTGROUP_MD]            =  (fsw_array[FSW_MD_FAULTS_INDEX] & FSW_MD_FAULTS_MASK) >> FSW_MD_FAULTS_SHIFT;
    faultGroup[FAULTGROUP_ARCHITECTURE]  =  (fsw_array[FSW_ARCH_FAULTS_INDEX] & FSW_ARCH_FAULTS_MASK) >> FSW_ARCH_FAULTS_SHIFT;
    faultGroup[FAULTGROUP_INTERNAL]      =  (fsw_array[FSW_INTERNAL_FAULTS_INDEX] & FSW_INTERNAL_FAULTS_MASK) >> FSW_INTERNAL_FAULTS_SHIFT;

    """
    MCU specific faults get a special category because there is more information
    """
    mcu_specific_group = [];
    mcu_specific_group.append(fsw_array[4]);
    mcu_specific_group.append(fsw_array[5]);
    mcu_specific_group.append(fsw_array[6]);
    mcu_specific_group.append(fsw_array[7]);

    """
    Create a master list of present faults then check all the bits in each
    faultgroup and go to the dictionaries to get the names
    """
    faults_present = [];

    for x in range(0,32):
        temp = int(math.pow(2,x))
        temp = (faultGroup[FAULTGROUP_CRITICAL] & temp);
        if (temp):
            faults_present.append(critical_fault_decode[temp]);

    for x in range(0,32):
        temp = int(math.pow(2,x))
        temp = (faultGroup[FAULTGROUP_COMM] & temp);
        if (temp):
            faults_present.append(comm_fault_decode[temp]);

    for x in range(0,32):
        temp = int(math.pow(2,x))
        temp = (faultGroup[FAULTGROUP_SENSORS] & temp);
        if (temp):
            faults_present.append(sensor_fault_decode[temp]);

    for x in range(0,32):
        temp = int(math.pow(2,x))
        temp = (faultGroup[FAULTGROUP_IMU] & temp);
        if (temp):
            faults_present.append(imu_fault_decode[temp]);

    for x in range(0,32):
        temp = int(math.pow(2,x))
        temp = (faultGroup[FAULTGROUP_MD] & temp);
        if (temp):
            faults_present.append(md_fault_decode[temp]);

    for x in range(0,32):
        temp = int(math.pow(2,x))
        temp = (faultGroup[FAULTGROUP_ARCHITECTURE] & temp);
        if (temp):
            faults_present.append(arch_fault_decode[temp]);

    for x in range(0,32):
        temp = int(math.pow(2,x))
        temp = (faultGroup[FAULTGROUP_INTERNAL] & temp);
        if (temp):
            faults_present.append(internal_fault_decode[temp]);


    return faults_present
