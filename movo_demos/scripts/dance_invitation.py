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

 \Platform: Ubuntu 16.04 LTS / ROS Kinetic
--------------------------------------------------------------------"""
import rospy
import roslib;
roslib.load_manifest('pocketsphinx')
roslib.load_manifest('sound_play')
from sound_play.libsoundplay import SoundClient

import roslaunch
import rospkg
import smach
import smach_ros
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult

from movo_msgs.msg import FaceFound

'''
Proposal solution of demo and comments for next step:

 - publish [pan_pose, tilt_pose, face_camera_z] of nearest_face via new topic in face_tracking.py when pan&tilt cmd are in the deadzone (already focused).
 - subscribe to [pan_pose, tilt_pose, face_camera_z] in MoveCloser, pan_pose for movo_base_rot, tilt_pose and face_camera_z for computing movo-people distance.
 - during movo_base motion, face_tracking should be on.
 - Combine SearchFace and MoveCloser as one state (MoveToPeople)

'''

class InitRobot(smach.State):
    def __init__(self, launch_tuck_robot, launch_move_to_origin, state_name):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self._launch_tuck_robot = launch_tuck_robot
        self._launch_move_to_origin = launch_move_to_origin
        self._state_name = state_name


    def execute(self, userdata):
        rospy.loginfo('Executing state ' + self._state_name)

        # tuck upper body joints
        self._launch_tuck_robot.start()
        rospy.sleep(3.0)
        self._launch_move_to_origin.start()

        move_base_result = rospy.wait_for_message('/movo_move_base/result', MoveBaseActionResult)
        if (move_base_result.status.text == 'Goal succeeded!'):
            rospy.logdebug('reached the initial pose! ')
            self._launch_tuck_robot.shutdown()
            self._launch_move_to_origin.shutdown()
            return 'succeeded'
        else:
            self._launch_tuck_robot.shutdown()
            self._launch_move_to_origin.shutdown()
            return 'failed'


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

        move_base_result = rospy.wait_for_message('/movo_move_base/result', MoveBaseActionResult)
        rospy.logdebug('Received the move_base result for target goal')
        if (move_base_result.status.text == 'Goal succeeded!'):
            self._launch_move_closer.shutdown()
            rospy.logdebug('reached the goal! ')
            return 'succeeded'
        else:
            self._launch_move_closer.shutdown()
            return 'failed'


class DanceRequest(smach.State):
    def __init__(self, launch_follow_me_pose, launch_invitation_answer, launch_face_detection):
        smach.State.__init__(self, outcomes=['accepted', 'rejected', 'failed'])
        self._launch_follow_me_pose = launch_follow_me_pose
        self._launch_invitation_answer = launch_invitation_answer
        self._launch_face_detection = launch_face_detection
        self._speech_pub = rospy.Publisher("/movo/voice/text", String, queue_size=1, latch=True)
        self._speech_text = String()


    def execute(self, userdata):
        rospy.loginfo('Executing state DANCE_REQUEST')
        self._launch_follow_me_pose.start()
        rospy.sleep(2)
        self._launch_invitation_answer.start()
        rospy.sleep(3)

        self._speech_text.data = "Please say yes movo to accept, or sorry movo to reject my invitation"
        rospy.loginfo("Please say yes movo to accept, or sorry movo to reject my invitation")
        self._speech_pub.publish(self._speech_text)
        rospy.sleep(5)

        self._launch_follow_me_pose.shutdown()

        max_try_time = 5
        tried_time = 0
        while not rospy.is_shutdown():
            tried_time += 1
            invitation_answer = rospy.wait_for_message('/asr_control/detected_words', String)
            invitation_answer.data = invitation_answer.data.lower()
            if invitation_answer.data.find("yes movo") > -1:
                self._speech_text.data = "Let the music rock!"
                rospy.loginfo("Let the music rock!")
                self._speech_pub.publish(self._speech_text)
                self._launch_invitation_answer.shutdown()
                rospy.sleep(5)
                return 'accepted'
            elif (invitation_answer.data.find("sorry movo") > -1):
                self._speech_text.data = "Remember you just turned off a dance invitation from a lovely robot. Bye bye then."
                rospy.loginfo("You said no")
                self._speech_pub.publish(self._speech_text)
                self._launch_face_detection.shutdown()
                self._launch_invitation_answer.shutdown()
                rospy.sleep(10)
                return 'rejected'
            else:
                self._speech_text.data = "I do not get your answer, can you repeat"
                self._speech_pub.publish(self._speech_text)
                rospy.sleep(5)
                if tried_time == max_try_time:
                    self._speech_text.data = "I tried %d times but still do not understand you. I am going back."%max_try_time
                    self._speech_pub.publish(self._speech_text)
                    self._launch_face_detection.shutdown()
                    self._launch_invitation_answer.shutdown()
                    rospy.sleep(5)
                    rospy.logwarn('failed to get answer with %d tries'%max_try_time)
                    return 'failed'


class LetsDance(smach.State):
    def __init__(self, launch_follow_me_activation, launch_face_detection):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self._launch_follow_me_activation = launch_follow_me_activation
        self._launch_face_detection = launch_face_detection

        # play music
        self._play_music_client = SoundClient()

        self._speech_pub = rospy.Publisher("/movo/voice/text", String, queue_size=1, latch=True)
        self._speech_text = String()

    def execute(self, userdata):
        rospy.loginfo('Executing state LETS_DANCE')
        try:
            self._play_music_client.playWave('/home/movo/movo_ws/src/movo_demos/launch/voice_control/we_are_robots.wav')
            self._launch_follow_me_activation.start()
        except:
            rospy.logwarn('failed to enable dance')
            return 'failed'

        music_duration = 20
        rospy.sleep(music_duration)

        self._speech_text.data = "You dance much better than Long fei. Thank you very much. I will take a rest now, good bye."
        self._speech_pub.publish(self._speech_text)
        rospy.sleep(5)
        self._launch_face_detection.shutdown()
        rospy.sleep(3)
        self._launch_follow_me_activation.shutdown()
        rospy.sleep(3)
        return 'succeeded'


class DanceInvitation():
    def __init__(self):
        rospy.on_shutdown(self._shutdown)

        # create roslaunch handles
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self._launch_tuck_robot = roslaunch.parent.ROSLaunchParent(uuid, [rospkg.RosPack().get_path('movo_demos') + '/launch/robot/tuck_robot.launch'])
        self._launch_tuck_robot2 = roslaunch.parent.ROSLaunchParent(uuid, [rospkg.RosPack().get_path('movo_demos') + '/launch/robot/tuck_robot.launch'])

        self._launch_move_to_origin = roslaunch.parent.ROSLaunchParent(uuid, [rospkg.RosPack().get_path('movo_demos') + '/launch/dance_invitation/move_to_origin.launch'])
        self._launch_move_to_origin2 = roslaunch.parent.ROSLaunchParent(uuid, [rospkg.RosPack().get_path('movo_demos') + '/launch/dance_invitation/move_to_origin.launch'])

        self._launch_face_detection = roslaunch.parent.ROSLaunchParent(uuid, [rospkg.RosPack().get_path('movo_demos') + '/launch/face_tracking/face_tracking.launch'])
        self._launch_move_closer = roslaunch.parent.ROSLaunchParent(uuid, [rospkg.RosPack().get_path('movo_demos') + '/launch/dance_invitation/move_closer.launch'])
        self._launch_follow_me_pose = roslaunch.parent.ROSLaunchParent(uuid, [rospkg.RosPack().get_path('movo_demos') + '/launch/follow_me/follow_me_pose.launch'])
        self._launch_invitation_answer = roslaunch.parent.ROSLaunchParent(uuid, [rospkg.RosPack().get_path('movo_demos') + '/launch/dance_invitation/invitation_answer.launch'])
        self._launch_follow_me_activation = roslaunch.parent.ROSLaunchParent(uuid, [rospkg.RosPack().get_path('movo_demos') + '/launch/follow_me/follow_me_activation.launch'])

        self._init_robot_obj = InitRobot(self._launch_tuck_robot, self._launch_move_to_origin, 'INIT_ROBOT')
        self._search_face_obj = SearchFace(self._launch_face_detection, self._launch_move_closer)
        self._move_closer_obj = MoveCloser(self._launch_move_closer)
        self._dance_request_obj = DanceRequest(self._launch_follow_me_pose, self._launch_invitation_answer, self._launch_face_detection)
        self._lets_dance_obj = LetsDance(self._launch_follow_me_activation, self._launch_face_detection)
        self._ending_back_obj = InitRobot(self._launch_tuck_robot2, self._launch_move_to_origin2, 'ENDING_BACK')

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
        self._launch_tuck_robot.shutdown()
        self._launch_move_to_origin.shutdown()
        self._launch_tuck_robot.shutdown()
        self._launch_tuck_robot2.shutdown()
        self._launch_face_detection.shutdown()
        self._launch_move_closer.shutdown()
        self._launch_follow_me_pose.shutdown()
        self._launch_invitation_answer.shutdown()
        self._launch_follow_me_activation.shutdown()
        # self._intro_spect.stop()
        pass


    def _construct_sm(self):
        rospy.loginfo('Constructing state machine')
        sm = smach.StateMachine(outcomes = ['succeeded', 'failed'])

        # create launch file handles
        with sm:
            smach.StateMachine.add('INIT_ROBOT', self._init_robot_obj, transitions={'succeeded': 'SEARCH_FACE', 'failed': 'failed'})
            smach.StateMachine.add('SEARCH_FACE', self._search_face_obj, transitions={'succeeded': 'MOVE_CLOSER', 'failed':'failed'})
            smach.StateMachine.add('MOVE_CLOSER', self._move_closer_obj, transitions={'succeeded': 'DANCE_REQUEST', 'failed':'failed'})
            smach.StateMachine.add('DANCE_REQUEST', self._dance_request_obj, transitions={'accepted': 'LETS_DANCE', 'rejected': 'ENDING_BACK', 'failed': 'ENDING_BACK'})
            smach.StateMachine.add('LETS_DANCE', self._lets_dance_obj, transitions={'succeeded': 'ENDING_BACK', 'failed': 'ENDING_BACK'})
            smach.StateMachine.add('ENDING_BACK', self._ending_back_obj, transitions={'succeeded': 'succeeded', 'failed': 'failed'})

        return sm


if __name__ == "__main__":
    rospy.loginfo('start dance invitation')
    rospy.init_node('dance_invitation', log_level=rospy.INFO)
    # rospy.init_node('dance_invitation', log_level=rospy.DEBUG)

    DanceInvitation()



