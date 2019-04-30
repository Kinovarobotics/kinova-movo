#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import speech_recognition as sr
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list
from si_utils.gripper_action_test import GripperActionTest
from si_utils.base_motion_test import BaseMotionTest
from trajectory_msgs.msg import (JointTrajectoryPoint,)
from si_utils.head_jtas_test import HeadJTASTest
from si_utils.voice_test import MovoVoiceTest
from si_utils.torso_jtas_test import TorsoJTASTest
from si_utils.jaco_jtas_test import JacoJTASTest
"""
This class takes root in the moveit_ Move Group Python Interface. This code is useful because someone can plan trajectories and movements with only a python code. The link [https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html].

It has been adapted for MOVO. 
"""

class Movo_movement(object): 
	"""
	Class used to control Movo in a friendly, non complicated way.
	"""
	def __init__(self, group_name = "head"):
		super(Movo_movement, self).__init__()
		# First, we initialize the node and moveit_commander
		
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
		self.movo_base = BaseMotionTest()

	def change_group(self, group_new): #Will change the group that is controlled with MoveIt.
		self.group = moveit_commander.MoveGroupCommander(group_new)
		self.planning_frame = self.group.get_planning_frame()
		self.eef_link = self.group.get_end_effector_link()
	#	
	def get_joint_state(self):
		return self.group.get_current_joint_values() 
		
	def get_current_pose(self):
		return self.group.get_current_pose(self.eef_link)
	def go_to_joint_state(self, coord): #Give coordinates to the arms for each actuators
		joint_goal = self.get_joint_state()

		for i in range(0,len(coord)):
			joint_goal[i]=coord[i]

		self.group.go(joint_goal, wait=True)
		self.group.stop()
		

	def change_joint_state_by_pos(self, pos, value): #Change one joint at a time by position of actuator (1-7)
		joint_goal = self.get_joint_state()
		joint_goal[pos]=value
		self.group.go(joint_goal, wait=True)
		self.group.execute()
		self.group.stop()
	
	def go_cartesian_pose(self,x, y, z, w):
		#Specifically for the arms. The group should be changed prior. Will move the end effector the that position.
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w =w
		pose_goal.position.x = x
		pose_goal.position.y = y
		pose_goal.position.z =z
		self.group.set_pose_target(pose_goal)
		plan = self.group.go(wait=True)
		self.group.stop()
		self.group.clear_pose_targets()
		
	def drive_to_pos(self,x,y,theta):	# Will move the robot to a certain position (note that it will not take the shortest route)
		if(x >0):
			self.movo_base.move_left(x)
		elif(x<0):
			self.movo_base.move_right(x)
		if(y> 0):
			self.movo_base.move_forward(y)
		elif(y<0):
			self.movo_base.move_backward(y)
		if(theta>0):
			self.movo_base.rotate_anticlock(theta)
		elif(theta<0):
			self.movo_base.rotate_clock(theta)
		self.current_base_position[0]+=x
		self.current_base_position[1]+=y
		self.current_base_position[2]+=theta
		self.movo_base.motion_stop()
	def go_home(self):		# Goes to its initial position.
		
		x = -1*self.current_base_position[0]
		y = -1*self.current_base_position[1]
		theta = -1*self.current_base_position[2]
		if(x >0):
			self.movo_base.move_left(x)
		elif(x<0):
			self.movo_base.move_right(x)
		if(y> 0):
			self.movo_base.move_forward(y)
		elif(y<0):
			self.movo_base.move_backward(y)
		if(theta>0):
			self.movo_base.rotate_anticlock(theta)
		elif(theta<0):
			self.movo_base.rotate_clock(theta)
		self.current_base_position[0] = 0
		self.current_base_position[1] = 0
		self.current_base_position[2] = 0
		self.movo_base.motion_stop()

def main():
	rospy.init_node('Movo_movement', anonymous=True)
	my_robot = Movo_movement('left_arm')	
	#my_robot.go_to_joint_state([0,0,0,0,0,0])
	my_robot.go_cartesian_pose(0.5, 0.3, 0.9, 0)
	my_robot.go_to_joint_state([1.57, 1.4, 2.5, 1.09, 0, 0])
	my_robot.go_to_joint_state([1.9, 0, 1.5, 1.09, 0, 0])
	my_robot.go_to_joint_state([1.1, 0, 1.5, 1.09, 0, 0])
	my_robot.go_to_joint_state([1.9, 0, 1.5, 1.09, 0, 0])
	my_robot.go_to_joint_state([1.1, 0, 1.5, 1.09, 0, 0])
	my_robot.go_to_joint_state([1.9, 0, 1.5, 1.09, 0, 0])
	my_robot.go_to_joint_state([1.57, 1.4, 2.5, 1.09, 0, 0])

if __name__== '__main__':
	main()
			
