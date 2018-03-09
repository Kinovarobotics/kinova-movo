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
 
 \file   movo_jtas.py

 \brief  This module offer an interface to control the movo arms

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from movo_joint_interface.jaco_joint_controller import SIArmController
from trajectory_smoother import TrajectorySmoother
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes

from control_msgs.msg import (
    FollowJointTrajectoryAction, 
    FollowJointTrajectoryFeedback, 
    FollowJointTrajectoryResult,
    GripperCommandAction,
    GripperCommandFeedback,
    GripperCommandResult,
)
                     
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import UInt16,Bool
from movo_msgs.msg import Status
from threading import Thread

import errno

import math
import rospy
import actionlib
import bisect
import operator
from copy import deepcopy

def calc_grip_dist(b):
    l1 = 30.9476-87.0932*math.sin(b[0]-0.627445866)
    l2 = 30.9476-87.0932*math.sin(b[1]-0.627445866)
    dist = l1+l2
    
    if (dist < (2*30.9476)):
        dist-=17.0
    else:
        dist+=1.08
    
    return (dist * 0.001)

def calc_grip_angle(x):
    
    dist = x*1000.0
    tmp = (0.5*dist-30.9476)/-87.0932
    a = math.asin(tmp)+0.627445866
    
    if (0.5*dist > 30.9476):
        a+=0.00599
    else:
        a-=0.1
    
    return (a)

class MovoArmJTAS(object):
    def __init__(self, prefix="", gripper="", interface='eth0', jaco_ip="10.66.171.15", dof="", rate=100.0):
        self._alive = False
        self.init_success = True

        self._action_name = rospy.get_name()
        self._prefix = prefix
        # Action Feedback/Result
        
        if ("kg2" == gripper):
            self.gripper_stall_force = 20.0
            self.gripper_dead_zone = 0.01
        elif("kg3" == gripper):
            self.gripper_stall_force = 30.0
            self.gripper_dead_zone = 0.01
            
        self._last_gripper_pos = 0.165
        self._gripper_stall_to = 0.7
        self._gripper_pos_stall = False
        self._last_movement_time = rospy.get_time()
        self.dof = dof
        self._planner_homing = False

        """
        Define the joint names
        """
        if ("6dof" == dof):
            self._joint_names = [self._prefix+'_shoulder_pan_joint',
                                 self._prefix+'_shoulder_lift_joint',
                                 self._prefix+'_elbow_joint',
                                 self._prefix+'_wrist_1_joint',
                                 self._prefix+'_wrist_2_joint',
                                 self._prefix+'_wrist_3_joint']

            self._body_joints = ["right_elbow_joint",
                                 "right_shoulder_lift_joint",
                                 "right_shoulder_pan_joint",
                                 "right_wrist_1_joint",
                                 "right_wrist_2_joint",
                                 "right_wrist_3_joint",
                                 "left_elbow_joint",
                                 "left_shoulder_lift_joint",
                                 "left_shoulder_pan_joint",
                                 "left_wrist_1_joint",
                                 "left_wrist_2_joint",
                                 "left_wrist_3_joint",
                                 "linear_joint",
                                 "pan_joint",
                                 "tilt_joint"]
            self._homed = [-2.135, -0.227, -1.478, -2.083, 1.445, 1.321, 2.135, 0.227, 1.478, 2.083, -1.445, -1.321, 0.25, 0.0, 0.0]

        elif ("7dof" == dof):
            self._joint_names = [self._prefix + '_shoulder_pan_joint',
                                 self._prefix + '_shoulder_lift_joint',
                                 self._prefix + '_arm_half_joint',
                                 self._prefix + '_elbow_joint',
                                 self._prefix + '_wrist_spherical_1_joint',
                                 self._prefix + '_wrist_spherical_2_joint',
                                 self._prefix + '_wrist_3_joint']

            self._body_joints = ["right_shoulder_pan_joint",
                                 "right_shoulder_lift_joint",
                                 "right_arm_half_joint",
                                 "right_elbow_joint",
                                 "right_wrist_spherical_1_joint",
                                 "right_wrist_spherical_2_joint",
                                 "right_wrist_3_joint",
                                 "left_shoulder_pan_joint",
                                 "left_shoulder_lift_joint",
                                 "left_arm_half_joint",
                                 "left_elbow_joint",
                                 "left_wrist_spherical_1_joint",
                                 "left_wrist_spherical_2_joint",
                                 "left_wrist_3_joint",
                                 "linear_joint",
                                 "pan_joint",
                                 "tilt_joint"]
            self._homed = [-1.5, -0.2, -0.15, -2.0, 2.0, -1.24, -1.1, 1.5, 0.2, 0.15, 2.0, -2.0, 1.24, 1.1, 0.25, 0, 0]

        else:
            rospy.logerr("DoF needs to be set 6 or 7, cannot start MovoArmJTAS")
            return

        """
        Controller parameters from arguments, messages, and dynamic
        reconfigure
        """
        self._trajectory_control_rate = rate  # Hz
        self._goal_time = 0.0
        self._stopped_velocity = 0.0
        self._goal_error = dict()
        self._path_thresh = dict()
        self._traj_smoother = TrajectorySmoother(rospy.get_name(),self._prefix)
        self._ctl = SIArmController(self._prefix,gripper,interface,jaco_ip, dof)
        self._ctl.Pause()
        self._estop_delay = 0
        self.home_arm_sub = rospy.Subscriber('/movo/home_arms', Bool, self._home_arms)
        self.home_arm_pub = rospy.Publisher('/movo/arms_are_homed', Bool, queue_size=1)
        self._arms_homing = False

        if not self._ctl.init_success:
            rospy.logerr("Failed to initialize controller, make sure the serial number exists")
            self.clean_shutdown()
            self.init_success = False
            return
        self.estop = False
        self._fdbk = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()        
        #self._dyn = reconfig_server
        self._ns = '/movo/%s_arm_controller'%self._prefix
        self._fjt_ns = self._ns + '/follow_joint_trajectory'
        self._server = actionlib.SimpleActionServer(
            self._fjt_ns,
            FollowJointTrajectoryAction,
            execute_cb=self._on_trajectory_action,
            auto_start=False)            
        self._alive = True
        self._movo_status_sub = rospy.Subscriber("/movo/feedback/status",Status,self._update_movo_status)
        self._server.start()
        
        # Action Server
        self._gripper_server = actionlib.SimpleActionServer(
            '/movo/%s_gripper_controller/gripper_cmd'%self._prefix,
            GripperCommandAction,
            execute_cb=self._on_gripper_action,
            auto_start=False)
        self._gripper_server.start()
        
        self._gripper_action_name = '/movo/%s_gripper_controller/gripper_cmd'%self._prefix

        # Action Feedback/Result
        self._gripper_fdbk = GripperCommandFeedback()
        self._gripper_result = GripperCommandResult()
        self._gripper_timeout = 6.0
        self._ctl.api.InitFingers()

    def _home_arm_planner(self):
        if self._prefix == 'left':
            rospy.sleep(5)
        else:
            move_group_jtas = MoveGroupInterface("upper_body", "base_link")
            move_group_jtas.setPlannerId("RRTConnectkConfigDefault")

            success = False
            while not rospy.is_shutdown() and not success:
                result = move_group_jtas.moveToJointPosition(self._body_joints, self._homed, 0.05)
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.logerr("_home_arm_planner completed ")
                    success = True
                else:
                    rospy.logerr("_home_arm_planner: _home_arm_planner failed (%d)" % result.error_code.val)

        self._arms_homing = True
        self._ctl.api.MoveHome()
        self._ctl.api.InitFingers()
        self.home_arm_pub.publish(Bool(True))
        rospy.sleep(2.0)
        self._arms_homing = False
        self._planner_homing = False

    def _update_gripper_feedback(self, position):
        tmp = self._ctl.GetGripperFdbk()
        
        grip_dist = calc_grip_dist(tmp[0])
        
        self._gripper_fdbk.position = grip_dist
        self._gripper_fdbk.effort = sum(tmp[2])
        
        self._gripper_fdbk.stalled = (self._gripper_fdbk.effort >
                              self.gripper_stall_force)
        self._gripper_fdbk.reached_goal = (math.fabs(grip_dist -
                                        position) <
                                   self.gripper_dead_zone)
        
        delta = math.fabs(self._gripper_fdbk.position - self._last_gripper_pos)
        self._last_gripper_pos = self._gripper_fdbk.position
        if (delta > 0.005):
            self._last_movement_time = rospy.get_time()
        
        if (rospy.get_time() - self._last_movement_time) > self._gripper_stall_to:
            self._gripper_pos_stall=True
        else:
            self._gripper_pos_stall=False
            
        self._gripper_fdbk.stalled |= self._gripper_pos_stall
            

        self._gripper_result = self._gripper_fdbk
        self._gripper_server.publish_feedback(self._gripper_fdbk)


    def _command_gripper(self, position):

        ang = calc_grip_angle(position)
        self._ctl.CommandGripper(ang)
        
    def _check_gripper_state(self):
        
        return (self._gripper_fdbk.stalled or self._gripper_fdbk.reached_goal)

    def _on_gripper_action(self, goal):
        # Store position and effort from call
        # Position to 0:0.165 == close:open
        position = goal.command.position
        effort = goal.command.max_effort
        print position

        # Reset feedback/result
        self._update_gripper_feedback(position)

        # 20 Hz gripper state rate
        control_rate = rospy.Rate(20.0)

        # Record start time
        start_time = rospy.get_time()

        def now_from_start(start):
            return rospy.get_time() - start

        # Continue commanding goal until success or timeout
        self._last_movement_time = rospy.get_time()
        self._last_gripper_pos = self._gripper_fdbk.position
        while ((now_from_start(start_time) < self._gripper_timeout or
               self._gripper_timeout < 0.0) and not rospy.is_shutdown()):
            if self._gripper_server.is_preempt_requested():
                self._ctl.StopGripper()
                rospy.loginfo("%s: Gripper Action Preempted" %
                              (self._gripper_action_name,))
                self._gripper_server.set_preempted(self._gripper_result)
                return
            self._update_gripper_feedback(position)
            if self._check_gripper_state():
                self._gripper_server.set_succeeded(self._gripper_result)
                return
            self._command_gripper(position)
            control_rate.sleep()

        # Gripper failed to achieve goal before timeout/shutdown
        self._ctl.StopGripper()
        if not rospy.is_shutdown():
            rospy.logerr("%s: Gripper Command Not Achieved in Allotted Time" %
                         (self._gripper_action_name,))
        self._update_gripper_feedback(position)
        self._gripper_server.set_aborted(self._gripper_result)
    
    def _home_arms(self,cmd):
        if (True == cmd.data and self._planner_homing == False):
            self._planner_homing = True
            b_thread = Thread(target=self._home_arm_planner(), args='')
            b_thread.daemon = True
            b_thread.start()

    def _update_movo_status(self,status):
        if (0 != status.dynamic_response) or (False == self._ctl.GetCtlStatus()) or self._arms_homing:
            self.estop = True
            self._ctl.SetEstop()
            self._estop_delay = 100
        else:
            if (0 == self._estop_delay):
                self.estop = False
                self._ctl.ClearEstop()
            else:
                self.estop = True
                self._ctl.SetEstop()
                self._estop_delay -= 1
    
    def robot_is_enabled(self):
        return not self.estop

    def clean_shutdown(self):
        self._ctl.Stop()
        self._alive = False

    def _get_trajectory_parameters(self, joint_names, goal):
        """
        For each input trajectory, if path, goal, or goal_time tolerances
        provided, we will use these as opposed to reading from the
        parameter server/dynamic reconfigure
        """

        """
        Goal time tolerance - time buffer allowing goal constraints to be met
        """
        if goal.goal_time_tolerance:
            self._goal_time = goal.goal_time_tolerance.to_sec()
        else:
            self._goal_time = 1.0
            
        """
        Stopped velocity tolerance - max velocity at end of execution
        """
        self._stopped_velocity = 0.5

        """
        Path execution and goal tolerances per joint
        """
        for jnt in joint_names:
            if jnt not in self._joint_names:
                rospy.logerr(
                    "%s: Trajectory Aborted - Provided Invalid Joint Name %s" %
                    (self._action_name, jnt,))
                self._result.error_code = self._result.INVALID_JOINTS
                self._server.set_aborted(self._result)
                return
                
            """
            Path execution tolerance
            """
            self._path_thresh[jnt] = 0.5
            if goal.path_tolerance:
                for tolerance in goal.path_tolerance:
                    if jnt == tolerance.name:
                        self._path_thresh[jnt] = tolerance.position
            
            """
            Goal error tolerance
            """
            self._goal_error[jnt] = 0.5
            if goal.goal_tolerance:
                for tolerance in goal.goal_tolerance:
                    if jnt == tolerance.name:
                        self._goal_error[jnt] = tolerance.position

    def _get_current_position(self, joint_names):
        return self._ctl.GetCurrentJointPosition(joint_names)

    def _get_current_velocities(self, joint_names):
        return self._ctl.GetCurrentJointVelocity(joint_names)

    def _get_current_errors(self, joint_names):
        error = self._ctl.GetCurrentJointPositionError(joint_names)
        return zip(joint_names, error)       

    def _update_feedback(self, cmd_point, joint_names, cur_time):
        self._fdbk.header.stamp = rospy.Duration.from_sec(rospy.get_time())
        self._fdbk.joint_names = joint_names
        self._fdbk.desired = cmd_point
        self._fdbk.desired.time_from_start = rospy.Duration.from_sec(cur_time)
        self._fdbk.actual.positions = self._get_current_position(joint_names)
        self._fdbk.actual.time_from_start = rospy.Duration.from_sec(cur_time)
        self._fdbk.error.positions = map(operator.sub,
                                         self._fdbk.desired.positions,
                                         self._fdbk.actual.positions
                                        )
        self._fdbk.error.time_from_start = rospy.Duration.from_sec(cur_time)
        self._server.publish_feedback(self._fdbk)

    def _command_stop(self): 
        self._ctl.SetPositionHold()
        self._ctl.ClearPositionHold()

    def _command_joints(self, joint_names, point, dimensions_dict):
        if self._server.is_preempt_requested() or not self.robot_is_enabled():
            rospy.loginfo("%s: Trajectory Preempted" % (self._action_name,))
            self._server.set_preempted()
            self._command_stop()
            return False

        deltas = self._get_current_errors(joint_names) 
        for delta in deltas:
            if ((math.fabs(delta[1]) >= self._path_thresh[delta[0]]
                and self._path_thresh[delta[0]] >= 0.0)) or not self.robot_is_enabled():
                rospy.logerr("%s: Exceeded Error Threshold on %s: %s" %
                             (self._action_name, delta[0], str(delta[1]),))
                self._result.error_code = self._result.PATH_TOLERANCE_VIOLATED
                self._server.set_aborted(self._result)
                self._command_stop()
                return False
                
        pos = dict(zip(joint_names, point.positions))
        vel = dict(zip(joint_names, [0.0]*len(joint_names)))
        acc = dict(zip(joint_names, [0.0]*len(joint_names)))
        if dimensions_dict['velocities']:
            vel = dict(zip(joint_names, point.velocities))
        if dimensions_dict['accelerations']:
            acc = dict(zip(joint_names, point.accelerations))
        
        if self._alive:
            self._ctl.CommandJoints(pos, vel, acc)
        return True
        
    def _check_goal_state(self, joint_names, last):
        for error in self._get_current_errors(joint_names):
            if (self._goal_error[error[0]] > 0
                    and self._goal_error[error[0]] < math.fabs(error[1])):
                return error[0]
        if (self._stopped_velocity > 0.0 and
            max([abs(cur_vel) for cur_vel in self._get_current_velocities(joint_names)]) >
                self._stopped_velocity):
            return False
        else:
            return True

    def _on_trajectory_action(self, goal):
        joint_names = goal.trajectory.joint_names
        self._get_trajectory_parameters(joint_names, goal)
        success,results = self._traj_smoother.ProcessTrajectory(goal.trajectory, 
                                                                self._get_current_position(joint_names),
                                                                False)
        if not success:
            self._server.set_aborted()
            return
            
        """
        Copy the results to variables that make sense namewise
        """
        dimensions_dict   = results[0] 
        b_matrix          = results[1]
        trajectory_points = results[2]
        pnt_times         = results[3]
        num_points        = results[4]
        
        
        """
        Wait for the specified execution time, if not provided use now
        """
        start_time = goal.trajectory.header.stamp.to_sec()
        now = rospy.get_time()
        if start_time == 0.0:
            start_time = rospy.get_time()
        while start_time > now:
            now = rospy.get_time()
        
        """
        Loop until end of trajectory time.  Provide a single time step
        of the control rate past the end to ensure we get to the end.
        Keep track of current indices for spline segment generation
        """
        self._ctl.Resume()
        control_rate = rospy.Rate(self._trajectory_control_rate)
        now_from_start = rospy.get_time() - start_time
        end_time = trajectory_points[-1].time_from_start.to_sec()
        while (now_from_start < end_time and not rospy.is_shutdown() and
               self.robot_is_enabled()):
            now = rospy.get_time()
            now_from_start = now - start_time
            idx = bisect.bisect(pnt_times, now_from_start)

            """
            Calculate percentage of time passed in this interval
            """
            if idx >= num_points:
                cmd_time = now_from_start - pnt_times[-1]
                t = 1.0
            elif idx >= 0:
                cmd_time = (now_from_start - pnt_times[idx-1])
                t = cmd_time / (pnt_times[idx] - pnt_times[idx-1])
            else:
                cmd_time = 0.0
                t = 0.0

            point = self._traj_smoother.GetBezierPoint(b_matrix, 
                                                       idx, 
                                                       t, 
                                                       cmd_time, 
                                                       dimensions_dict)

            """
            Command Joint Position, Velocity, Acceleration
            """
            command_executed = self._command_joints(joint_names, point, dimensions_dict)
            self._update_feedback(deepcopy(point), joint_names, now_from_start)

            """
            Break the loop if the command cannot be executed
            """
            if not command_executed:
                return
            control_rate.sleep()
            
        """
        Keep trying to meet goal until goal_time constraint expired
        """
        last = trajectory_points[-1]
        last_time = trajectory_points[-1].time_from_start.to_sec()
        end_angles = dict(zip(joint_names, last.positions))
        while (now_from_start < (last_time + self._goal_time)
               and not rospy.is_shutdown() and self.robot_is_enabled()):
            if not self._command_joints(joint_names, last, dimensions_dict):
                return
            now_from_start = rospy.get_time() - start_time
            self._update_feedback(deepcopy(last), joint_names,
                                  now_from_start)
            control_rate.sleep()

        now_from_start = rospy.get_time() - start_time
        self._update_feedback(deepcopy(last), joint_names,
                                  now_from_start)

        """
        Verify goal constraint
        """
        result = self._check_goal_state(joint_names, last)
        
        if result is True:
            rospy.loginfo("%s: Joint Trajectory Action Succeeded for %s arm" %
                          (self._action_name, self._prefix))
            self._result.error_code = self._result.SUCCESSFUL
            self._server.set_succeeded(self._result)
        elif result is False:
            rospy.logerr("%s: Exceeded Max Goal Velocity Threshold for %s arm" %
                         (self._action_name, self._prefix))
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(self._result)
        else:
            rospy.logerr("%s: Exceeded Goal Threshold Error %s for %s arm" %
                         (self._action_name, result, self._prefix))
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(self._result)
        self._command_stop()
        self._ctl.Pause()
