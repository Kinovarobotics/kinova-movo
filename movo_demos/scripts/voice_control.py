#!/usr/bin/env python

"""
voice_cmd_vel.py is a simple demo of speech recognition.
  You can control a mobile base using commands found
  in the corpus file.
"""

import roslib;

roslib.load_manifest('pocketsphinx')
import rospy
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class voice_cmd_vel:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.speed = 0.2
        self.msg = Twist()

        # publish to cmd_vel, subscribe to speech output
        self.pub_ = rospy.Publisher('cmd_vel', Twist)
        rospy.Subscriber('recognizer/output', String, self.speechCb)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.pub_.publish(self.msg)
            r.sleep()

    def speechCb(self, msg):
        rospy.loginfo(msg.data)

        if msg.data.find("forward") > -1:
            rospy.loginfo("I heard forward")
        elif msg.data.find("left") > -1:
            rospy.loginfo("I heard left")
        elif msg.data.find("right") > -1:
            rospy.loginfo("I heard right")
        elif msg.data.find("stop") > -1 or msg.data.find("halt") > -1:
            self.msg = Twist()

        self.pub_.publish(self.msg)

    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.pub_.publish(twist)


if __name__ == "__main__":
    rospy.init_node('voice_cmd_vel')
    try:
        voice_cmd_vel()
    except:
        pass

