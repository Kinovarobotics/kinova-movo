#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list
from trajectory_msgs.msg import (JointTrajectoryPoint,)

"""
This class takes root in the moveit_ Move Group Python Interface. This code is useful because someone can plan trajectories and movements with only a python code. The link [https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html].

It has been adapted for MOVO.
"""

class Movo_movement(object): 
	"""
	Self made class used to control Movo in a friendly, non complicated way.
	"""
	def __init__(self, group_name = "head"):
		super(Movo_movement, self).__init__()
		rospy.init_node('Movo_movement', anonymous=True)
		moveit_commander.roscpp_initialize(sys.argv)
		# We instantiate a robotCommander
		movo = moveit_commander.RobotCommander()
		# We create a scene, in case we want to visualize that scene in Rviz
		scene = moveit_commander.PlanningSceneInterface()
		# We then initialize our MoveGroupCommander, the group we are going to modify
		group = moveit_commander.MoveGroupCommander(group_name)
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
		self.box_name =''
		self.movo = movo
		self.group = group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = group.get_planning_frame()
		self.eef_link = group.get_end_effector_link()
		self.group_names = movo.get_group_names()
		self.current_base_position = [0,0,0] #[x,y,theta]

	def get_current_pose(self):
		return self.group.get_current_pose(self.eef_link) #Will return the current position of the end effector
	
	def go_cartesian_pose(self,x, y, z, w):
		#Specifically for the arms. The group should be changed prior. (left of right)
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w =w
		pose_goal.position.x = x
		pose_goal.position.y = y
		pose_goal.position.z =z
		self.group.set_pose_target(pose_goal)
		plan = self.group.go(wait=True)
		self.group.stop()
		self.group.clear_pose_targets()
		

def main():

	my_robot = Movo_movement('left_arm')
	position = my_robot.get_current_pose()
	print position
	my_robot.go_cartesian_pose(0.805, 0.11, 1.047, 0.57)
	

if __name__== '__main__':
	main()
			
