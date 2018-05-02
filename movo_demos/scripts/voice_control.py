#!/usr/bin/env python
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

 \file   voice_control

 \Author Longfei Zhao

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import threading
import numpy

import roslib;
roslib.load_manifest('pocketsphinx')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class voice_control:

    def __init__(self):
        rospy.on_shutdown(self._shutdown)
        self._movo_base_cmd_vel = Twist()
        self._movo_base_Vx = 0.2
        self._movo_base_Vy = 0.2
        self._movo_base_Rz = numpy.radians(30.0)
        self._movo_base_cmd_duration = 5.0


        # subscriber
        self._speech_sub = rospy.Subscriber('recognizer/output', String, self._speechCb)
        self._latest_command_time = rospy.get_rostime()

        # publisher
        self._movo_base_cmd_pub = rospy.Publisher('/movo/base/voice_control/cmd_vel', Twist, queue_size = 10)

        # publisher thread
        self._movo_base_cmd_mutex = threading.Lock()
        self._movo_base_cmd_thread = threading.Thread(target=self._thread_run)
        self._movo_base_cmd_thread.start()

        rospy.loginfo('Voice control initialized')
        rospy.spin()

    def _speechCb(self, msg):
        rospy.loginfo(msg.data)

        with self._movo_base_cmd_mutex:
            self._latest_command_time = rospy.get_rostime()
            if msg.data.find("forward") > -1:
                self._movo_base_cmd_vel.linear.x = self._movo_base_Vx
                self._movo_base_cmd_vel.linear.y = 0.0
            elif msg.data.find("backward") > -1:
                self._movo_base_cmd_vel.linear.x = -self._movo_base_Vx
                self._movo_base_cmd_vel.linear.y = 0.0
            elif msg.data.find("left") > -1:
                self._movo_base_cmd_vel.linear.x = 0.0
                self._movo_base_cmd_vel.linear.y = self._movo_base_Vy
            elif msg.data.find("right") > -1:
                self._movo_base_cmd_vel.linear.x = 0.0
                self._movo_base_cmd_vel.linear.y = -self._movo_base_Vy
            elif msg.data.find("stop") > -1 or msg.data.find("halt") > -1:
                self._movo_base_cmd_vel.linear.x = 0.0
                self._movo_base_cmd_vel.linear.y = 0.0
            else:
                self._movo_base_cmd_vel.linear.x = 0.0
                self._movo_base_cmd_vel.linear.y = 0.0


    def _thread_run(self):
        rospy.loginfo('voice control movo base command publisher thread is running')
        while not rospy.is_shutdown():
            with self._movo_base_cmd_mutex:
                if (rospy.get_rostime() - self._latest_command_time).to_sec() <= self._movo_base_cmd_duration:
                    self._movo_base_cmd_pub.publish(self._movo_base_cmd_vel)


    def _shutdown(self):
        # stop the robot!
        self._movo_base_cmd_vel.linear.x = 0.0
        self._movo_base_cmd_vel.linear.y = 0.0
        self._movo_base_cmd_pub.publish(self._movo_base_cmd_vel)
        pass


if __name__ == "__main__":
    rospy.init_node('voice_control')
    try:
        rospy.loginfo('Initializing voice control')
        voice_control()
    except:
        rospy.logerr("voice control initialized failed")
        pass

