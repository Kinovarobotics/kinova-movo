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
 
 \file   movo_torso_jtas.py

 \brief  This module

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from movo_msgs.msg import LinearActuatorCmd,Status
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import UInt16,Bool
from sensor_msgs.msg import JointState

import errno
import math
import rospy
import actionlib
import bisect
import operator
from copy import deepcopy
import cmd



class MovoTorsoJTAS(object):
    def __init__(self, rate=100.0):
        self._alive = False
        self.init_success = False

        self._action_name = rospy.get_name()
        # Action Feedback/Result

        """
        Define the joint names
        """
        self._joint_names = ['linear_joint']

        """
        Controller parameters from arguments, messages, and dynamic
        reconfigure
        """
        self._trajectory_control_rate = rate  # Hz
        self._goal_time = 0.0
        self._stopped_velocity = 0.0
        self._goal_error = dict()
        self._path_thresh = dict()
        self._estop_delay = 0

        self._fdbk = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()        
        #self._dyn = reconfig_server
        self._ns = '/movo/torso_controller'
        self._fjt_ns = self._ns + '/follow_joint_trajectory'
        self._server = actionlib.SimpleActionServer(
            self._fjt_ns,
            FollowJointTrajectoryAction,
            execute_cb=self._on_trajectory_action,
            auto_start=False)            
        self._alive = True
        self.estop = False
        self._movo_status_sub = rospy.Subscriber("/movo/feedback/status",Status,self._update_movo_status)
        self._js = rospy.wait_for_message("/movo/linear_actuator/joint_states",JointState)
        self.pos_targets = self._get_current_position(self._joint_names)
        self._sub = rospy.Subscriber("/movo/linear_actuator/joint_states",JointState,self._update_movo_joint_states)
        self._cmd_pub=rospy.Publisher("/movo/linear_actuator_cmd",LinearActuatorCmd,queue_size=10)
        self._server.start()
        
        self.init_success=True
    
    def _update_movo_status(self,status):
    
        
        if (0 != status.dynamic_response):
            self.estop = True
        else:
            self.estop = False
            
    def _update_movo_joint_states(self,msg):
        self._js = msg
    
    def robot_is_enabled(self):
        return not self.estop

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
        pos = dict(zip(self._js.name,self._js.position))
        pos = [pos[jnt] for jnt in joint_names]
        return pos

    def _get_current_velocities(self, joint_names):
        vel = dict(zip(self._js.name,self._js.velocity))
        vel = [vel[jnt] for jnt in joint_names]
        return vel

    def _get_current_errors(self, joint_names):
        actual = self._get_current_position(joint_names)
        error = map(operator.sub,self.pos_targets,actual)
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
        """
        NEED STOP FUNCTION
        """
        self.pos_targets = self._get_current_position(self._joint_names)

    def _command_joints(self, joint_names, point):
        if self._server.is_preempt_requested() or not self.robot_is_enabled():
            rospy.loginfo("%s: Trajectory Preempted" % (self._action_name,))
            self._server.set_preempted()
            self._command_stop()
            return False

        self.pos_targets = point.positions
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
                
        cmdz = LinearActuatorCmd()
        cmdz.desired_position_m = point.positions[0]
        
        if self._alive:
            self._cmd_pub.publish(cmdz)
            actual = self._get_current_position(joint_names)
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
        
        num_joints = len(joint_names)
        trajectory_points = goal.trajectory.points
        pnt_times = [0.0]*len(trajectory_points)
        # Create a new discretized joint trajectory
        num_points = len(trajectory_points)
        if num_points == 0:
            rospy.logerr("%s: Empty Trajectory" % (action_name,))
            self._server.set_aborted()
            return
            
        # Force Velocites/Accelerations to zero at the final timestep
        # if they exist in the trajectory
        # Remove this behavior if you are stringing together trajectories,
        # and want continuous, non-zero velocities/accelerations between
        # trajectories
        trajectory_points[-1].velocities = [0.0] * len(joint_names)
        trajectory_points[-1].accelerations = [0.0] * len(joint_names)

        # Compute Full Bezier Curve Coefficients for all 7 joints
        pnt_times = [pnt.time_from_start.to_sec() for pnt in trajectory_points]
            
        if (num_points == 1) or (pnt_times[0] > 0.0):
            # Add current position as trajectory point
            first_trajectory_point = JointTrajectoryPoint()
            first_trajectory_point.positions = self._get_current_position(joint_names)
            # To preserve desired velocities and accelerations, copy them to the first
            # trajectory point if the trajectory is only 1 point.
            first_trajectory_point.time_from_start = rospy.Duration(0)
            trajectory_points.insert(0, first_trajectory_point)
            num_points = len(trajectory_points)
        
        
        
        for i in range(num_points):
            trajectory_points[i].velocities = [0.0] * len(joint_names)
            trajectory_points[i].accelerations = [0.0] * len(joint_names)
            
        for i in range(1,num_points):
            for j in range(num_joints):
                if ((pnt_times[i] - pnt_times[i-1]) > 0.0):
                    trajectory_points[i].velocities[j] = (trajectory_points[i].positions[j]-trajectory_points[i-1].positions[j])/(pnt_times[i] - pnt_times[i-1])
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
        control_rate = rospy.Rate(self._trajectory_control_rate)
        now_from_start = rospy.get_time() - start_time
        end_time = trajectory_points[-1].time_from_start.to_sec()
        last_idx = 0
        slewed_pos = self._get_current_position(joint_names)
        self._command_joints(joint_names, deepcopy(trajectory_points[0]))
        while (now_from_start < end_time and not rospy.is_shutdown() and
               self.robot_is_enabled()):
            now = rospy.get_time()
            now_from_start = now - start_time
            
            for i in range(num_points):
                if (pnt_times[i] >= now_from_start):
                    idx = i
                    break
            
            if (idx >= num_points):
                idx = num_points-1
                                
            for j in range(num_joints):
                slewed_pos[j] += trajectory_points[idx].velocities[j] * (1.0/self._trajectory_control_rate)

            point = deepcopy(trajectory_points[idx])
            point.positions = slewed_pos
            
            """
            Command Joint Position, Velocity, Acceleration
            """
            command_executed = self._command_joints(joint_names, point)
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
            if not self._command_joints(joint_names, last):
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
            rospy.loginfo("%s: Joint Trajectory Action Succeeded" %self._action_name)
            self._result.error_code = self._result.SUCCESSFUL
            self._server.set_succeeded(self._result)
        elif result is False:
            rospy.logerr("%s: Exceeded Max Goal Velocity Threshold" %self._action_name)
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(self._result)
        else:
            rospy.logerr("%s: Exceeded Goal Threshold Error %s"%(self._action_name, result))
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(self._result)
        self._command_stop()
