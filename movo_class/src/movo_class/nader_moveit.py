#!/usr/bin/env python
"""
This code is meant to control MOVO with voice commands using a map created prior to the demo. In short, when I tell it to go somewhere, for example Nassif's office, it should go there.
"""

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
import tf
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, PointStamped, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import *
import rospkg


goals = { "My Office": [3.54, 4.2, 0.000, 1],"Ralph's office": [-4.112, 5.416, 0.000, 1.385],"Kitchen": [-5.792, 5.701, 0.000, 3.049]}
class Goto:
	def __init__(self):
		self.goalpub_= rospy.Publisher("/move_base_simple/goal", PoseStamped,queue_size=10)
		self.msg = PoseStamped()
		self.msg.header.frame_id ="map"
		self.cmd_vel_pub = rospy.Publisher("/movo/teleop/cmd_vel", Twist, queue_size=10)			
	def pubMsg(self, goal):
		self.msg.pose.position.x = goals[goal][0]
		self.msg.pose.position.y = goals[goal][1]
		self.msg.pose.position.z = goals[goal][2]
		self.msg.pose.orientation.w =goals[goal][3]
        self.goalpub_.publish(self.msg)
	def cleanup(self):
		twist = Twist()
        self.cmd_vel_pub.publish(twist)

if __name__=="__main__":
	rospy.init_node('goto')
	GOMOVO=Goto()
	GOMOVO.pubMsg("My Office")


