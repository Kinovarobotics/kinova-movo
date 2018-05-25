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

'''
Proposal solution of demo and comments for next step:

 - publish [pan_pose, tilt_pose, face_camera_z] of nearest_face via new topic in face_tracking.py when pan&tilt cmd are in the deadzone (already focused).
 - subscribe to [pan_pose, tilt_pose, face_camera_z] in MoveCloser, pan_pose for movo_base_rot, tilt_pose and face_camera_z for computing movo-people distance.
 - during movo_base motion, face_tracking should be on.
 - Combine SearchFace and MoveCloser as one state (MoveToPeople)

'''


class SearchFace(smach.State):
    def __init__(self, launch_face_detection):
        smach.State.__init__(self, outcomes=['succeeded'])
        self._launch_face_detection = launch_face_detection

    def execute(self, userdata):
        rospy.loginfo('Executing state SEARCH_FACE')

        self._launch_face_detection.start()
        rospy.sleep(20)

        return 'succeeded'


class MoveCloser(smach.State):
    def __init__(self, launch_face_detection):
        smach.State.__init__(self, outcomes=['succeeded'])
        self._launch_face_detection = launch_face_detection

    def execute(self, userdata):
        rospy.loginfo('Executing state MOVE_CLOSER')

        self._launch_face_detection.shutdown()
        rospy.sleep(2)
        return 'succeeded'


class DanceInvitation():
    def __init__(self):
        rospy.on_shutdown(self._shutdown)

        # create roslaunch handles
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_face_detection = roslaunch.parent.ROSLaunchParent(uuid, [rospkg.RosPack().get_path('movo_demos') + '/launch/face_tracking/face_tracking.launch'])

        self._SearchFace_obj = SearchFace(launch_face_detection)
        self._MoveCloser_obj = MoveCloser(launch_face_detection)

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
        # self._intro_spect.stop()
        pass


    def _construct_sm(self):
        rospy.loginfo('Constructing state machine')
        sm = smach.StateMachine(outcomes = ['succeeded'])

        # create launch file handles
        with sm:
            smach.StateMachine.add('SEARCH_FACE', self._SearchFace_obj, transitions={'succeeded': 'MOVE_CLOSER'})
            smach.StateMachine.add('MOVE_CLOSER', self._MoveCloser_obj, transitions={'succeeded': 'succeeded'})

        return sm


if __name__ == "__main__":
    rospy.loginfo('start dance invitation')
    rospy.init_node('dance_invitation', log_level=rospy.INFO)
    # rospy.init_node('dance_invitation', log_level=rospy.DEBUG)

    DanceInvitation()



