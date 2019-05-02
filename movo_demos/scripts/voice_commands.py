#!/usr/bin/env python
"""
This code is meant to control MOVO with voice commands using a map created prior to the demo. In short, when I tell it to go somewhere, for example 'main office', it should go there.
"""

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PointStamped, PoseWithCovarianceStamped, Point, Quaternion, Twist
from std_msgs.msg import String, Bool
from si_utils.gripper_action_test import GripperActionTest
from si_utils.voice_test  import MovoVoiceTest
from si_utils.base_motion_test import BaseMotionTest
#These should be included in voice_input.py
commands= ["stop", "open", "close", "thank", "hi","start", "left", "right", "forward", "back"]
class voice_commands:
	def __init__(self):
		rospy.init_node('voice_commands')
		self.gripper = GripperActionTest("left") #We only control the left arm
		self.voice_sub = rospy.Subscriber('/Microphone/speech_recognition/', String, self.speechReceiver)
		self.cmd_vel_pub = rospy.Publisher('/movo/manual_override/cmd_vel', Twist, queue_size=10)
		self.on=True
		self.movo_voice = MovoVoiceTest()
		self.base_motion= BaseMotionTest()
	def speechReceiver(self, voice):
		rospy.loginfo(voice.data)		
		for i in commands:
			if i in voice.data:
				if i =="stop":
					self.on=False
					self.cleanup()
				elif i == "open":
					self.gripper.command(0.165)
					self.gripper.wait()
				elif i == "close":
					print "got it"
					self.gripper.command(0)
					self.gripper.wait()
				elif i == "thank":
					self.movo_voice.say("My pleasure")
					rospy.sleep(2)
				elif i == "hi":
					self.movo_voice.say("Hi, my name is Movo.")
					rospy.sleep(2)
				elif i =="start":
					self.on=True
				elif i =="left":
					self.base_motion.move_left(0.2)
				elif i =="right":
					self.base_motion.move_right(0.2)
				elif i =="forward":
					self.base_motion.move_forward(0.2)
				elif i =="back":
					self.base_motion.move_backward(0.2)
					
	def checkState(self):
		return self.on
	def cleanup(self):
		r = rospy.Rate(10)
		start_time = rospy.get_time()
		_movo_base_cmd_vel=Twist()
		_movo_base_cmd_vel.linear.x = 0.0
        	_movo_base_cmd_vel.linear.y = 0.0
        	_movo_base_cmd_vel.angular.z = 0.0
		while ((rospy.get_time()-start_time) <10.0):
			self.cmd_vel_pub.publish(_movo_base_cmd_vel)
			r.sleep()

if __name__=="__main__":
	m=voice_commands()
	rospy.spin()
	
	
