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
from movo_demos.gripper_action_test import GripperActionTest
from movo_demos.base_motion_test import BaseMotionTest
from trajectory_msgs.msg import (JointTrajectoryPoint,)
from movo_demos.head_jtas_test import HeadJTASTest
from movo_demos.voice_test import MovoVoiceTest
from movo_demos.torso_jtas_test import TorsoJTASTest
from movo_demos.jaco_jtas_test import JacoJTASTest
"""
This class takes root in the moveit_ Move Group Python Interface. This code is useful because someone can plan trajectories and movements with only a python code. The link [https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html].

It has been adapted for MOVO, or at least, it should work on MOVO. 
"""

class Movo_movement(object): 
	"""
	Self made class used to control Movo in a friendly, non complicated way.
	"""
	def __init__(self, group_name = "head"):
		super(Movo_movement, self).__init__()
		# First, we initialize the node and moveit_commander
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
		self.movo_base = BaseMotionTest()

	def change_group(self, group_new): #works
		self.group = moveit_commander.MoveGroupCommander(group_new)
		self.planning_frame = self.group.get_planning_frame()
		self.eef_link = self.group.get_end_effector_link()
		
	def get_joint_state(self):
		return self.group.get_current_joint_values()
	#look for global variable	
	def get_current_pose(self):
		return self.group.get_current_pose(self.eef_link)
	def get_torque(self):
		torque= rospy.wait_for_message("/movo/right_arm/joint_states", JointState).effort
		return torque
	def go_to_joint_state(self, coord): #works
		joint_goal = self.get_joint_state()

		for i in range(0,len(coord)):
			joint_goal[i]=coord[i]

		self.group.go(joint_goal, wait=True)
		self.group.stop()
		#return all_close(joint_goal, self.group.get_current_joint_values(), 0.01)

	def change_joint_state_by_pos(self, pos, value): #works
		joint_goal = self.get_joint_state()
		joint_goal[pos]=value
		self.group.go(joint_goal, wait=True)
		self.group.stop()
	
	def go_cartesian_pose(self,x, y, z, w):
		#Specifically for the arms. The group should be changed prior.
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.orientation.w =w
		pose_goal.position.x = x
		pose_goal.position.y = y
		pose_goal.position.z =z
		self.group.set_pose_target(pose_goal)
		plan = self.group.go(wait=True)
		self.group.stop()
		self.group.clear_pose_targets()
		#return all_close(pose_goal, self.group.get_current_pose().pose, 0.01)
	def drive_to_pos(self,x,y,theta):
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
	def go_home(self):
		
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
	def listen(self):
		r =sr.Recognizer()
		with sr.Microphone() as source:
			print("Say something")
			audio =r.listen(source)
		try:
			print("You said: " +r.recognize_google(audio))
		except sr.UnknownValueError:
			print("Google Speech Recognition could not understand audio")
		except sr.RequestError as e:
			print("Could not request results from Google Speech Recognition service; {0}".format(e))		   
		

def main():
	#lg=GripperActionTest("left")
	mon_robot = Movo_movement('left_arm')	
	#mon_robot.go_cartesian_pose(-0.218,0.8399,0.8653,0.61)
	#lg.command(1.16)
	#lg.stop()

if __name__== '__main__':
	main()
			
