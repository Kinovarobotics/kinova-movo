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

--------------------------------------------------------------------------"""

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from gripper_action_test import GripperActionTest

def movo_moveit_test():
    ## BEGIN_TUTORIAL
    ##
    ## Setup
    ## ^^^^^
    ## CALL_SUB_TUTORIAL imports
    ##
    ## First initialize moveit_commander and rospy.
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('movo_moveit_test',
                  anonymous=True)
    
    is_sim = rospy.get_param('~sim',False)

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()
    
    
    ##Gripper action clients
    lgripper = GripperActionTest('left')
    rgripper = GripperActionTest('right')


    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    group = moveit_commander.MoveGroupCommander("upper_body")


    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,queue_size=10)

    rospy.sleep(2)
    scene.remove_world_object("floor")
    
    # publish a demo scene
    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.0
    p.pose.position.y = 0.0
    p.pose.position.z = -0.01
    p.pose.orientation.w = 1.0
    scene.add_box("floor", p, (4.0, 4.0, 0.02))
    

    ## We can get a list of all the groups in the robot
    print "============ Robot Groups:"
    print robot.get_group_names()

    ## Sometimes for debugging it is useful to print the entire state of the
    ## robot.
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"
    
    group.set_planner_id("RRTConnectkConfigDefault")

    
    ## Planning to a Pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the 
    ## end-effector
    if (True == is_sim):
        gripper_closed = 0.96
        gripper_open = 0.0
    else:
        gripper_closed = 0.0
        gripper_open = 0.165        

    
    print "============ Testing with named targets"
    group.set_named_target("homed")
    plan = group.plan()
    group.execute(plan)
    lgripper.command(gripper_open)
    rgripper.command(gripper_open)
    lgripper.wait()
    rgripper.wait()
    
    rospy.sleep(2.0) 
    
    group.set_named_target("tucked")
    plan = group.plan()
    group.execute(plan)
    lgripper.command(gripper_closed)
    rgripper.command(gripper_closed)
    lgripper.wait()
    rgripper.wait()

    rospy.sleep(2.0) 
    
    group.set_named_target("pos1")
    plan = group.plan()
    group.execute(plan)
    lgripper.command(gripper_open)
    rgripper.command(gripper_open)
    lgripper.wait()
    rgripper.wait()

    rospy.sleep(2.0) 

    group.set_named_target("tucked")
    plan = group.plan()
    group.execute(plan)
    lgripper.command(gripper_closed)
    rgripper.command(gripper_closed)
    lgripper.wait()
    rgripper.wait()    

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

    ## END_TUTORIAL

    print "============ STOPPING"


if __name__=='__main__':
  try:
    movo_moveit_test()
  except rospy.ROSInterruptException:
    pass

