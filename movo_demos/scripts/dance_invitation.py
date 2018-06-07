#!/usr/bin/env python
"""--------------------------------------------------------------------
Copyright (c) 2018, Kinova Robotics inc.

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

 \file   dance invitagion

 \Author Longfei Zhao

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
import rospy
import roslaunch
import rospkg
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseActionResult

from movo_msgs.msg import FaceFound

'''
Proposal solution of demo and comments for next step:

 - publish [pan_pose, tilt_pose, face_camera_z] of nearest_face via new topic in face_tracking.py when pan&tilt cmd are in the deadzone (already focused).
 - subscribe to [pan_pose, tilt_pose, face_camera_z] in MoveCloser, pan_pose for movo_base_rot, tilt_pose and face_camera_z for computing movo-people distance.
 - during movo_base motion, face_tracking should be on.
 - Combine SearchFace and MoveCloser as one state (MoveToPeople)

'''


class SearchFace(smach.State):
    def __init__(self, launch_face_detection, launch_move_closer):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self._launch_face_detection = launch_face_detection
        self._launch_move_closer = launch_move_closer

    def execute(self, userdata):
        rospy.loginfo('Executing state SEARCH_FACE')

        self._launch_face_detection.start()
        rospy.sleep(5)
        self._launch_move_closer.start()
        rospy.sleep(5)

        rospy.wait_for_message("/face_detector/nearest_face", FaceFound)
        return 'succeeded'


class MoveCloser(smach.State):
    def __init__(self, launch_move_closer):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self._launch_move_closer = launch_move_closer


    def execute(self, userdata):
        rospy.loginfo('Executing state MOVE_CLOSER')

        # two goals: first to go to init pose, second to go to target_pose

        move_base_result = rospy.wait_for_message('/movo_move_base/result', MoveBaseActionResult)
        if move_base_result.header.seq == 1:
            rospy.logdebug('Received the move_base result for initial pose goal')
            move_base_result = rospy.wait_for_message('/movo_move_base/result', MoveBaseActionResult)

        if move_base_result.header.seq == 2:
            rospy.logdebug('Received the move_base result for target goal')
            if (move_base_result.status.text == 'Goal succeeded!'):
                rospy.logdebug('reached the goal! ')
                return 'succeeded'
            else:
                return 'failed'


class DanceRequest(smach.State):
    def __init__(self, launch_follow_me_pose):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self._launch_follow_me_pose = launch_follow_me_pose


    def execute(self, userdata):
        rospy.loginfo('Executing state DanceRequest')
        self._launch_follow_me_pose.start()
        rospy.sleep(5)
        return 'succeeded'


class DanceInvitation():
    def __init__(self):
        rospy.on_shutdown(self._shutdown)

        # create roslaunch handles
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self._launch_face_detection = roslaunch.parent.ROSLaunchParent(uuid, [rospkg.RosPack().get_path('movo_demos') + '/launch/face_tracking/face_tracking.launch'])
        self._launch_move_closer = roslaunch.parent.ROSLaunchParent(uuid, [rospkg.RosPack().get_path('movo_demos') + '/launch/dance_invitation/move_closer.launch'])
        self._launch_follow_me_pose = roslaunch.parent.ROSLaunchParent(uuid, [rospkg.RosPack().get_path('movo_demos') + '/launch/follow_me/follow_me_pose.launch'])

        self._search_face_obj = SearchFace(self._launch_face_detection, self._launch_move_closer)
        self._move_closer_obj = MoveCloser(self._launch_move_closer)
        self._dance_request_obj = DanceRequest(self._launch_follow_me_pose)

        # construct state machine
        self._sm = self._construct_sm()
        rospy.loginfo('State machine Constructed')

        # Run state machine introspection server for smach viewer
        self._intro_spect = smach_ros.IntrospectionServer('dance_invitation', self._sm, '/DANCE_INVITATION')
        self._intro_spect.start()
        outcome = self._sm.execute()

        rospy.spin()
        self._intro_spect.stop()


    def _shutdown(self):
        self._launch_face_detection.shutdown()
        self._launch_move_closer.shutdown()
        self._launch_follow_me_pose.shutdown()
        # self._intro_spect.stop()
        pass


    def _construct_sm(self):
        rospy.loginfo('Constructing state machine')
        sm = smach.StateMachine(outcomes = ['succeeded', 'failed'])

        # create launch file handles
        with sm:
            smach.StateMachine.add('SEARCH_FACE', self._search_face_obj, transitions={'succeeded': 'MOVE_CLOSER', 'failed':'failed'})
            smach.StateMachine.add('MOVE_CLOSER', self._move_closer_obj, transitions={'succeeded': 'DANCE_REQUEST', 'failed':'failed'})
            smach.StateMachine.add('DANCE_REQUEST', self._dance_request_obj, transitions={'succeeded': 'succeeded', 'failed': 'failed'})

        return sm


if __name__ == "__main__":
    rospy.loginfo('start dance invitation')
    rospy.init_node('dance_invitation', log_level=rospy.INFO)
    # rospy.init_node('dance_invitation', log_level=rospy.DEBUG)

    DanceInvitation()



