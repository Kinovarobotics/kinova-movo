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
--------------------------------------------------------------------"""

import rospy
import tf
import actionlib
from actionlib_msgs.msg import *
from movo_msgs.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import Pose2D
from movo.system_defines import *
from helpers import Helpers
from math import sqrt,pow

def trunc(f, n):
    """
    Truncates/pads a float f to n decimal places without rounding
    """
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen]) 


class MoveBaseActionClient(object):
    def __init__(self,frame="map",sim=False):
        self.client_initialized = False
        self. _helpers = Helpers()
        self.n_successes = 0
        self.n_goals = 0
        self.running_time = 0.0
        self.distance_traveled = 0.0
        self.last_pose = self._helpers.GetCurrentRobotPose(frame)
        self.start_time = rospy.get_time()
        
        self._frame = frame
        
        """
        Subscribe to the move_base action server
        """
        self.move_base_client = actionlib.SimpleActionClient("/movo_move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server")
    
        """
        Wait 60 seconds for the action server to become available
        """
        if (self.move_base_client.wait_for_server(rospy.Duration(10))):
            rospy.loginfo("Connected to move base server")
        else:
            rospy.logerr("Could not connect to action server")
            self._shutdown()
            return
        
        rospy.sleep(1)

        """
        Make sure the robot is in tractor mode so it can accept motion commands
        """
        if not sim:
            if (False == self. _helpers.SetRobotMode(TRACTOR_REQUEST)):
                rospy.logerr("Could not set operational state")
                rospy.logerr("Platform did not respond")
                self._shutdown()
                return
        
        self.client_initialized = True
    
    def __del__(self):
        self.move_base_client.cancel_all_goals()
        
    def goto(self, point2d, timeout=300.0):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = point2d.x
        move_goal.target_pose.pose.position.y = point2d.y
        
        rot = tf.transformations.quaternion_from_euler(0,0,point2d.theta)
        move_goal.target_pose.pose.orientation.x = rot[0]
        move_goal.target_pose.pose.orientation.y = rot[1]
        move_goal.target_pose.pose.orientation.z = rot[2]
        move_goal.target_pose.pose.orientation.w = rot[3]
        move_goal.target_pose.header.frame_id = self._frame
        move_goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base_client.send_goal(move_goal,done_cb=self._done_moving_cb)
        self.n_goals +=1

        goal_status = self.move_base_client.wait_for_result(rospy.Duration(timeout))
        
        if not goal_status:
            self.move_base_client.cancel_all_goals()
            rospy.logerr("Did not complete the navigation goal within the timeout period specified")
        
        return goal_status
        
    def cancel(self):
        self.move_base_client.cancel_all_goals()
        
    def _done_moving_cb(self,status,result):

        if status == GoalStatus.SUCCEEDED:
            self.n_successes += 1
        elif status == GoalStatus.ABORTED:
            rospy.loginfo("Goal aborted with error code: " + str(self.goal_states[status])) 
        elif status != GoalStatus.PREEMPTED:
            rospy.loginfo("Goal preemted with error code: " + str(self.goal_states[status])) 

        new_pose = self._helpers.GetCurrentRobotPose(self._frame)
        self.distance_traveled += sqrt(pow(new_pose.pose.pose.position.x - 
                            self.last_pose.pose.pose.position.x, 2) +
                        pow(new_pose.pose.pose.position.y - 
                            self.last_pose.pose.pose.position.y, 2))
        self.last_pose = new_pose

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
        
    
