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

\Author Longfei Zhao
\brief  Demo prepared for Movo2
\Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""

import roslib; roslib.load_manifest('face_detector')
import rospy
from sensor_msgs.msg import PointCloud
from people_msgs.msg import PositionMeasurementArray
import face_detector.msg
# from movo_action_clients.head_action_client import HeadActionClient


class FaceTracking:
    def __init__(self):
        self.candidate_id = []
        self.valid_face = False
        self.enable_tracking = False

        # # set running rate as 0.1s for face tracking.
        # rate = rospy.Rate(0.1)

        self.face_detector_sub = rospy.Subscriber("/face_detector/faces_cloud", PointCloud, self._face_tracking)
        # self.face_detector_sub = rospy.Subscriber("/face_detector/people_tracker_measuremes_array", PositionMeasurementArray, self._face_tracking)
        print ("FaceTracking initialization")

        # rate.sleep()
        rospy.spin()


    # this call back function is trigged on each time detect a face.
    # Otherwise, subscriber do not hear message from the topic /face_detector/faces_cloud/

    def _face_tracking(self, msg):

        print("Receive face information")
        # if(rospy.Time.now().to_sec() - pointCloudData.header.stamp.to_sec() < 1):
        #     head_pose_to_camera = pointCloudData.points
        #     print head_pose_to_camera

        # movo_head = HeadActionClient()

if __name__ == "__main__":
    print ("start the node")
    rospy.init_node("face_detection")

    # start face tracking
    FaceTracking()



