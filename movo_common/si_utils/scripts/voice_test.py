#!/usr/bin/env python

"""------------------------------------------------------------------------------------
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

------------------------------------------------------------------------------------"""

import rospy
from std_msgs.msg import String

class MovoVoiceTest(object):

    def __init__(self):
        self.voice_cmd = String()
        self._pub = rospy.Publisher("/movo/voice/text", String, queue_size=1, latch=True)

    def say(self, data):
        self.voice_cmd.data = data
        self._pub.publish(self.voice_cmd)
        """
        rospy.sleep(3.0)
        self._pub.unregister()
        self._pub = rospy.Publisher("/movo/voice/text", String, queue_size=1, latch=True)    
        """

if __name__ == "__main__":
    rospy.init_node('voice_test')
    movo_voice = MovoVoiceTest()

    rospy.sleep(2)


    movo_voice.say("Hello there. My name is MOVO. "
                   "Welcome to IROS. You can get to know me, and all I can do, over the next minute and a half. "
                   "It will be fun. ")
    rospy.sleep(15)

    movo_voice.say("I can move all over the place. Forward... and backward... and to the side. "
                   "Do I look like Michael Jackson? ")
    rospy.sleep(15)

    movo_voice.say("There are certain advantages to being a robot. We do not have to worry about being too tall or too short. "
                   "Just right for whatever we need to do. ")
    rospy.sleep(15)

    movo_voice.say("Thanks to six degrees of freedom, I can move my arms in many ways. "
                   "It is very helpful for all sorts of tasks. And I run on open source, open architecture. I am ROS enabled. "
                   "Please join my BETA program and help me to be the best I-can be. ")
    rospy.sleep(20)

    movo_voice.say("I have three fingers that are capable for thousands of tasks. ")
    rospy.sleep(10)

    movo_voice.say("People say I have nice eyes. I can look up at the sky or down at the ground. "
                   "I have two laser sensors on my base, and they are great for SLAM application too. ")
    rospy.sleep(15)

    movo_voice.say("If you would like to participate in my BETA phase, talk to the guys at the booth "
                   "or visit KinovaMOVO.com for more details. "
                   "Remember my name MOVO. You will be hearing a lot about me in the future. I love you guys. ")
    rospy.sleep(20)

    movo_voice.say("Now, please allow me to take a little bit rest. I will be back to see you in 5 minutes. Good bye. ")