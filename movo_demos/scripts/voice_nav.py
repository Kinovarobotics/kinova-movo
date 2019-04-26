#!/usr/bin/env python
"""
This code is meant to control MOVO with voice commands using a map created prior to the demo. In short, when I tell it to go somewhere, for example Nassif's office, it should go there.
"""

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PointStamped, PoseWithCovarianceStamped, Point, Quaternion, Twist
import os
from std_msgs.msg import String, Bool
#The goals are predetermined positions in a map with a location name as key. They need to be changed to work with a specific map.

goals = { "boss": [-3.86, 5.36, 0.000, 1],"friend": [-3.98, -3.54, 0.000, 1], "hallway":[-4.529,0.1,0.0,1.0], "home": [-0.42,0.1, 0.0,1.0], "lobby":[4.36,-4.96,0,1.0]} #test_zone
#goals = { "boss": [-5.0, 2.34, 0.000, 0],"friend": [0.04, 2.18, 0.000, -1.385], "hallway":[-6.66,0.18,0.0,1.0], "home": [-2.44,-5.826, 0.0,1.385], "lobby":[-6.8,0.28,0,0]} #soussol2
#goals = { "boss": [15.647, -7.48, 0.000, 1.0],"friend": [-6.515, -13.64, 0.000, 1.385], "hallway":[-4.17,-16.13,0.0,1.0], "home": [0.7807,-7.05, 0.0,1.385], "lobby":[0,0,0,0]} #ff_1
#goals = { "boss": [-5.5, -15.988, 0.000, 1.0],"ralph": [-4.112, 5.416, 0.000, 1.385],"kitchen": [-5.792, 5.701, 0.000, 3.049], "hallway":[-0.58,-7.52,0.0,1.0]}
class Goto:
	def __init__(self):
		rospy.init_node('voice_nav')
		self.goalpub= rospy.Publisher('/move_base_simple/goal', PoseStamped,queue_size=10)
		self.msg = PoseStamped()
		self.state = False
		self.msg.header.frame_id ="map"
		self.voice_command=""
		self.cmd_stop = rospy.Publisher('/movo/teleop/abort_navigation',Bool, queue_size=10)
		self.voice_sub = rospy.Subscriber('/Microphone/speech_recognition/', String, self.speechReceiver)
	def speechReceiver(self, voice):
		rospy.loginfo(voice.data)		
		for i,j in goals.items():
			if i == voice.data:
				self.state=True
				self.pubMsg(i)

	def pubMsg(self, goal):
		self.msg.pose.position.x = goals[goal][0]
		self.msg.pose.position.y = goals[goal][1]
		self.msg.pose.position.z = goals[goal][2]
		self.msg.pose.orientation.w =goals[goal][3]
		self.goalpub.publish(self.msg)
		self.state=False

if __name__=="__main__":
	m=Goto()
	rospy.spin()
	
	
