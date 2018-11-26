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

 \file   move_base.py

 \brief  runs the driver

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import rospy
import tf
import actionlib
from system_defines import *
from actionlib_msgs.msg import *
from movo_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, PointStamped, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import *
from std_msgs.msg import Bool, UInt32
from math import pow, sqrt
from system_defines import *
from visualization_msgs.msg import MarkerArray,Marker
from math import atan2
import rospkg

class MovoMoveBase():
    def __init__(self):

        """
        Initialize parameters and flags
        """
        self.continue_execution = True
        self.movo_battery_low = False
        self.movo_issued_dyn_rsp = False
        self.is_sim = rospy.get_param("~sim", False)
        self.using_amcl = rospy.get_param("~using_amcl", False)
        self.global_frame = rospy.get_param("~global_frame", 'odom')
        self.base_frame = rospy.get_param("~base_frame", 'base_link')
        self.goal_timeout_sec = rospy.get_param("~goal_timeout_sec", 300)
        self.load_waypoints = rospy.get_param("~load_waypoints", False)
        self.waypoint_dwell_s= rospy.get_param("~waypoints_dwell_time", 5.0)
        self.run_waypoints = rospy.get_param("~run_waypoints", False)
        self.max_markers = 100
        self._init_markers()
        self.marker_array_pub = rospy.Publisher('/movo/waypoints',MarkerArray,queue_size=10)

        

        rospack = rospkg.RosPack()
        self.goals_filename = rospack.get_path('movo_demos') + "/goals/" + rospy.get_param("~goalfile", "movo_goals")  + ".txt"

        """
        Goal state return values
        """
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        self.waypoints = []
        self.present_waypoint = 0
        if (True == self.load_waypoints):
            goalfile = open(self.goals_filename,'r')
            for line in goalfile:
                goal = [float(i) for i in line.strip('\n').split(',')]
                pose = Pose(Point(goal[0], goal[1], goal[2]), Quaternion(goal[3],goal[4],goal[5],goal[6]))
                self._append_waypoint_pose(pose)
            goalfile.close()

        """
        Variables to keep track of success rate, running time,
        and distance traveled
        """
        self.n_goals = 0
        self.n_successes = 0
        self.distance_traveled = 0
        self.start_time = rospy.get_time()
        self.running_time = 0
        self.movo_operational_state = 0
        initial_mode_req = TRACTOR_REQUEST


        """
        Initialize subscribers
        """
        rospy.Subscriber("/movo/feedback/battery", Battery, self._handle_low_aux_power)
        rospy.Subscriber("/movo/feedback/status", Status, self._handle_status)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped,  self._simple_goal_cb)
        rospy.Subscriber('/movo/teleop/abort_navigation',Bool,self._shutdown)
        rospy.Subscriber('/clicked_point',PointStamped,self._add_waypoint)
        rospy.Subscriber('/movo/teleop/record_pose',Bool,self._add_waypoint_pose)
        rospy.Subscriber('/movo/waypoint_cmd',UInt32,self._process_waypoint_cmd)
        self.simple_goal_pub = rospy.Publisher('/movo_move_base/goal', MoveBaseActionGoal, queue_size=10)
        self.new_goal = MoveBaseActionGoal()

        """
        Publishers to manually control the robot (e.g. to stop it) and send gp commands
        """
        self.config_cmd = ConfigCmd()
        self.cmd_config_cmd_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/movo/teleop/cmd_vel', Twist, queue_size=10)

        if (False == self.is_sim):
            if (False == self._goto_mode_and_indicate(initial_mode_req)):
                rospy.logerr("Could not set operational state")
                rospy.logerr("Platform did not respond")
                self._shutdown()
                return

            """
            Get the initial pose from the user
            """
            if (True == self.using_amcl):
                rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
                rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)

                my_cmd = Twist()
                my_cmd.angular.z = 0.31415
                start_time = rospy.get_time()
                r = rospy.Rate(20)
                while (rospy.get_time() - start_time) < 10.0:
                    self.cmd_vel_pub.publish(my_cmd)
                    r.sleep()

            my_cmd = Twist()
            my_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(my_cmd)

        self.last_pose = self._get_current_pose()

        if (None == self.last_pose):
            rospy.logerr('Could not get initial pose!!!! exiting....')
            self._shutdown()
            return

        """
        Subscribe to the move_base action server
        """
        self.move_base_client = actionlib.SimpleActionClient("move_base_navi", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...move_base_navi")

        """
        Wait 60 seconds for the action server to become available
        """
        if (self.move_base_client.wait_for_server(rospy.Duration(60))):
            rospy.loginfo("Connected to move base server")
        else:
            rospy.logerr("Could not connect to action server")
            self._shutdown()
            return

        """
        Start the action server
        """
        self.action_ = MoveBaseAction()
        self.move_base_server = actionlib.SimpleActionServer("movo_move_base", MoveBaseAction,execute_cb=self._execute_goal, auto_start = False)
        self.move_base_server.register_preempt_callback(self._preempt_cb)
        self.move_base_server.start()

        rospy.loginfo("Movo move base server started")
        self.waypoint_is_executing = False
        self._run_waypoints()

    def _run_waypoints(self):
        rospy.sleep(5)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if ((len(self.waypoints) > 0) and (self.present_waypoint < len(self.waypoints)) and (False == self.waypoint_is_executing) and (True == self.run_waypoints)):
                self.waypoint_is_executing = True
                goal = PoseStamped()
                goal.header.stamp = rospy.get_rostime()
                goal.header.frame_id = self.global_frame
                if ((True == self.waypoints[self.present_waypoint][0]) and (len(self.waypoints)>1)) :
                    pos1 = self.waypoints[self.present_waypoint][1]
                    
                    if (self.present_waypoint == (len(self.waypoints)-1)):
                        pos2 = self.waypoints[0][1]
                    else:
                        pos2 = self.waypoints[self.present_waypoint+1][1]    
                    
                    y2y1= pos2.position.y-pos1.position.y
                    x2x1= pos2.position.x-pos1.position.x
                    heading = tf.transformations.quaternion_from_euler(0,0,atan2(y2y1,x2x1))
                    self.waypoints[self.present_waypoint][1].orientation.x = heading[0]
                    self.waypoints[self.present_waypoint][1].orientation.y = heading[1]
                    self.waypoints[self.present_waypoint][1].orientation.z = heading[2]
                    self.waypoints[self.present_waypoint][1].orientation.w = heading[3]
                    
                goal.pose = self.waypoints[self.present_waypoint][1] 

                self._simple_goal_cb(goal)

            self.marker_array_pub.publish(self.marker_array_msg)
            r.sleep()

    def _init_markers(self):
        self.marker_idx = 0
        self.marker_array_msg = MarkerArray()
        for i in range(self.max_markers):
            marker = Marker()
            marker.header.frame_id = self.global_frame
            marker.id = self.marker_idx
            marker.type = 2
            marker.action = 2
            marker.pose = Pose()
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.frame_locked = False
            marker.ns = "Goal-%u"%i
            self.marker_array_msg.markers.append(marker)

    def _process_waypoint_cmd(self,cmd):
        cmd = cmd.data
        rospy.loginfo("cmd rcvd %u"%cmd)
        if (1<<0 == cmd):
            self._add_waypoint_pose()
        elif (1<<1 == cmd):
            self.run_waypoints = True
        elif (1<<2 == cmd):
            self.run_waypoints = False
            if (True == self.waypoint_is_executing):
                self.move_base_client.cancel_goal()
                self.move_base_server.set_aborted(None, "User stopped waypoints")
                rospy.loginfo("User commanded waypoint record to stop")
        elif (1<<3 == cmd):
            self.run_waypoints = False
            self.present_waypoint = 0
            if (True == self.waypoint_is_executing):
                self.move_base_client.cancel_goal()
                self.move_base_server.set_aborted(None, "User reset waypoint record")
                rospy.loginfo("User commanded waypoint record to reset")
            for i in range(self.max_markers):
                self.marker_array_msg.markers[i].color.r = 1.0
                self.marker_array_msg.markers[i].color.g = 0.0
        elif (1<<4 == cmd):
            self.waypoints = []
            self._init_markers()
            self.present_waypoint = 0

        elif (1<<5 == cmd):
            self.waypoints = []
            self._init_markers()
            self.present_waypoint = 0

            goalfile = open(self.goals_filename,'r')
            for line in goalfile:
                goal = [float(i) for i in line.strip('\n').split(',')]
                pose = Pose(Point(goal[0], goal[1], goal[2]), Quaternion(goal[3],goal[4],goal[5],goal[6]))
                self._append_waypoint_pose(pose)
            goalfile.close()
        elif (1<<6 == cmd):
            goalfile = open(self.goals_filename,'w')
            for pose in self.waypoints:
                goal  = "%.3f,"%pose.position.x
                goal += "%.3f,"%pose.position.y
                goal += "%.3f,"%pose.position.z
                goal += "%.3f,"%pose.orientation.x
                goal += "%.3f,"%pose.orientation.y
                goal += "%.3f,"%pose.orientation.z
                goal += "%.3f\n"%pose.orientation.w
                goalfile.write(goal)
            goalfile.close()
            rospy.loginfo("Waypoint Record Saved: %s"%self.goals_filename)


    def _add_waypoint(self,point):
        pose = Pose(point.point,Quaternion(0.0,0.0,0.0,1.0))
        self._append_waypoint_pose(pose,True) 

    def _add_waypoint_pose(self):
        current_pose = self._get_current_pose()

        if (None != current_pose):
            self._append_waypoint_pose(current_pose.pose.pose)
        else:
            rospy.logerror("Invalid waypoint pose")

    def _append_waypoint_pose(self,pose,create_heading=False):
        if (self.marker_idx > self.max_markers):
            rospy.logwarn("You have exceeded the maximum number of allowed waypoints.....")
            return
        self.waypoints.append([create_heading,pose])
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.id = self.marker_idx
        marker.type = 2
        marker.action = 0
        marker.pose = pose
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.frame_locked = False
        marker.ns = "Goal-%u"%self.marker_idx
        self.marker_array_msg.markers[self.marker_idx] = marker
        self.marker_idx+=1


    def _execute_goal(self,goal):

        rospy.loginfo("Received a new goal")

        """
        See if the battery is low (threshold is 10% ABB reports at 5%)
        TODO: check FSW from embedded system
        """
        if self.movo_battery_low:
            rospy.loginfo("Dangerous to navigate with low state of charge, Runtime Warning..... Plug me in to charge.")
            return


        """
        Send the goal; Allow user defined timeout to get there;Let the user know where the robot is going next
        """
        rospy.loginfo("Going to (X,Y): (%(1).3f,%(2).3f)"%{"1":goal.target_pose.pose.position.x,"2":goal.target_pose.pose.position.y})

        self.n_goals+=1
        self.goal_timeout = rospy.Duration(self.goal_timeout_sec)
        self.goal_start_time = rospy.get_time()
        self.move_base_client.send_goal(goal,done_cb=self._done_moving_cb,feedback_cb=self._feedback_cb)
        delay = rospy.Duration(0.1)

        while not self.move_base_client.wait_for_result(delay) and not rospy.is_shutdown():
            """
            If the battery is low, we timed out, or got preempted stop moving
            """
            if self.movo_battery_low:
                self.move_base_client.cancel_goal()
                self.move_base_server.set_aborted(None, "Dangerous to navigate with low state of charge, cancelling goal")
                rospy.loginfo("Dangerous to navigate with low state of charge, Runtime Warning... Plug me in to charge..")
                return

            if self.movo_issued_dyn_rsp:
                self.move_base_client.cancel_goal()
                self.move_base_server.set_aborted(None, "Platform initiated dynamic response")
                rospy.loginfo("Cannot navigate when platform is executing dynamic response")
                return

            if ((rospy.get_time() - self.goal_start_time) > self.goal_timeout.to_sec()):
                self.move_base_client.cancel_goal()
                self.move_base_server.set_aborted(None, "Goal has timed out took longer than %f"%self.goal_timeout)
                rospy.loginfo("Timed out while trying to acheive new goal, cancelling move_base goal.")
                return

        """
        The goal should not be active at this point
        """
        assert not self.move_base_server.is_active()

    def _feedback_cb(self,feedback):
        self.move_base_server.publish_feedback(feedback)

    def _preempt_cb(self):
        self.move_base_client.cancel_goals_at_and_before_time(rospy.get_rostime())
        rospy.logwarn("Current move base goal cancelled")
        if (self.move_base_server.is_active()):
            if not self.move_base_server.is_new_goal_available():
                rospy.loginfo("Preempt requested without new goal, cancelling move_base goal.")
                self.move_base_client.cancel_goal()

            self.move_base_server.set_preempted(MoveBaseResult(), "Got preempted by a new goal")


    def _done_moving_cb(self,status,result):

        if status == GoalStatus.SUCCEEDED:
            self.n_successes += 1
            self._moving = False
            self.move_base_server.set_succeeded(result, "Goal succeeded!")
        elif status == GoalStatus.ABORTED:
            self.move_base_server.set_aborted(result, "Failed to move, ABORTED")
            rospy.loginfo("Goal aborted with error code: " + str(self.goal_states[status]))
        elif status != GoalStatus.PREEMPTED:
            self.move_base_server.set_aborted(result, "Unknown result from move_base")
            rospy.loginfo("Goal failed with error code: " + str(self.goal_states[status]))


        new_pose = self._get_current_pose()
        self.distance_traveled += sqrt(pow(new_pose.pose.pose.position.x -
                            self.last_pose.pose.pose.position.x, 2) +
                        pow(new_pose.pose.pose.position.y -
                            self.last_pose.pose.pose.position.y, 2))
        self.last_pose = new_pose

        sleep_time = 0
        if (True == self.waypoint_is_executing):
            self.marker_array_msg.markers[self.present_waypoint].color.g = 1.0
            self.marker_array_msg.markers[self.present_waypoint].color.r = 0.0
            self.present_waypoint+=1
            self.waypoint_is_executing = False

            sleep_time = self.waypoint_dwell_s

        """
        How long have we been running?
        """
        self.running_time = rospy.get_time() - self.start_time
        self.running_time = self.running_time / 60.0

        """
        Print a summary success/failure, distance traveled and time elapsed
        """
        rospy.loginfo("Success so far: " + str(self.n_successes) + "/" +
                      str(self.n_goals) + " = " +
                      str(100 * self.n_successes/self.n_goals) + "%")
        rospy.loginfo("Running time: " + str(trunc(self.running_time, 1)) +
                      " Total Distance: " + str(trunc(self.distance_traveled, 1)) + " m")

        if (sleep_time > 0):
            rospy.sleep(sleep_time)

    def _simple_goal_cb(self, simple_goal):

        """
        Make sure the goal is in the global reference frame before adding it to the queue;
        sometimes the user can have the wrong frame selected in RVIZ for the fixed frame
        It should usually be /map or /odom depending on how the user is running the navigation stack
        """
        if (simple_goal.header.frame_id != self.global_frame) and (('/'+simple_goal.header.frame_id) != self.global_frame):
            rospy.logerr('MoveBaseSimpleGoal is not in correct frame!!!')
            rospy.logerr('expected global frame %(1)s but got %(2)s'%{'1':self.global_frame,'2':simple_goal.header.frame_id})
            return

        self.new_goal.goal.target_pose = simple_goal
        self.simple_goal_pub.publish(self.new_goal)

    def _handle_low_aux_power(self, battery_msg ):
        if (battery_msg.battery_soc < 10.0):
            self.movo_battery_low = True

    def _handle_status(self,stat):
        if stat.dynamic_response != 0:
            self.movo_issued_dyn_rsp = True

        self.movo_operational_state = stat.operational_state

    def _goto_mode_and_indicate(self,requested):
        """
        define the commands for the function
        """
        config_cmd = ConfigCmd()

        """
        Send the mode command
        """
        r = rospy.Rate(10)
        start_time = rospy.get_time()
        while ((rospy.get_time() - start_time) < 30.0) and (MOVO_MODES_DICT[requested] != self.movo_operational_state):
            config_cmd.header.stamp = rospy.get_rostime()
            config_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
            config_cmd.gp_param = requested
            self.cmd_config_cmd_pub.publish(config_cmd)
            r.sleep()

        if (MOVO_MODES_DICT[requested] != self.movo_operational_state):
            rospy.logerr("Could not set operational Mode")
            rospy.loginfo("The platform did not respond, ")
            return False

    def _get_current_pose(self):

        """
        Gets the current pose of the base frame in the global frame
        """
        current_pose = None
        listener = tf.TransformListener()
        rospy.sleep(1.0)
        try:
            listener.waitForTransform(self.global_frame, self.base_frame, rospy.Time(), rospy.Duration(1.0))
        except:
            pass
        try:
            (trans,rot) = listener.lookupTransform(self.global_frame, self.base_frame, rospy.Time(0))

            pose_parts = [0.0] * 7
            pose_parts[0]  = trans[0]
            pose_parts[1]  = trans[1]
            pose_parts[2]  = 0.0
            euler = tf.transformations.euler_from_quaternion(rot)
            rot = tf.transformations.quaternion_from_euler(0,0,euler[2])
            pose_parts[3] = rot[0]
            pose_parts[4] = rot[1]
            pose_parts[5] = rot[2]
            pose_parts[6] = rot[3]

            current_pose = PoseWithCovarianceStamped()
            current_pose.header.stamp = rospy.get_rostime()
            current_pose.header.frame_id = self.global_frame
            current_pose.pose.pose = Pose(Point(pose_parts[0], pose_parts[1], pose_parts[2]), Quaternion(pose_parts[3],pose_parts[4],pose_parts[5],pose_parts[6]))
        except:
            rospy.loginfo("Could not get transform from %(1)s->%(2)s"%{"1":self.global_frame,"2":self.base_frame})

        return current_pose


    def _shutdown(self):
        rospy.loginfo("Stopping the robot...")
        try:
            self.move_base_client.cancel_all_goals()
        except:
            pass

        try:
            r = rospy.Rate(10)
            start_time = rospy.get_time()
            while ((rospy.get_time() - start_time) < 2.0):
                self.cmd_vel_pub.publish(Twist())
                r.sleep()
        except:
            pass

def trunc(f, n):
    """
    Truncates/pads a float f to n decimal places without rounding
    """
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])
