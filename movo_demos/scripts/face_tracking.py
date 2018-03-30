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

import numpy as np
from operator import attrgetter # min of list of class attributes

class Face:
    def __init__(self, detected_face = None):
        if(detected_face == None):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.dist = np.linalg.norm(np.array([self.x, self.y, self.z]))
        else:
            self.x = detected_face.x
            self.y = detected_face.y
            self.z = detected_face.z
            self.dist = np.linalg.norm(np.array([self.x, self.y, self.z]))
        # self.valid = False
        self.id = 0 # useful when subscribed to PositionMeasurementArray

    def update(self, detected_face):
        # if detected_face.id == self.id
        self.x = detected_face.x
        self.y = detected_face.y
        self.z = detected_face.z
        self.dist = np.linalg.norm(np.array([self.x, self.y, self.z]))

    def update_by_xyz(self, x, y, z):
        # if detected_face.id == self.id
        self.x = x
        self.y = y
        self.z = z
        self.dist = np.linalg.norm(np.array([self.x, self.y, self.z]))


class FaceTracking:
    def __init__(self):
        self.list_faces = []
        self.nearest_face = Face()

        # just for testing
        self.fakeface = Face()
        self.fakeface.update_by_xyz(1, 2, 3)

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

        # clear the list of founded faces in this moment
        del self.list_faces[:]

        print "=============================================="

        # when detected face is bad, face_detector send msg with empty pose []
        if(len(msg.points) == 0):
            print("no valid face detected")

        # valid face found
        else:
            for detected_face in msg.points:
                self.list_faces.append(Face(detected_face))
            self.list_faces.append(self.fakeface)

            # extract the face which is nearest to the camera frame
            # dist_nearest_face = min(self.list_faces, key=lambda x: x.dist).dist
            dist_nearest_face = min(face.dist for face in self.list_faces)

            # idx_nearest_face is one element list
            idx_nearest_face = [index for index in range(len(self.list_faces)) if self.list_faces[index].dist == dist_nearest_face]
            print "the nearest face has index ", idx_nearest_face, ", distance to camera is ", dist_nearest_face
            print "self.list_faces[idx_nearest_face].x is ", self.list_faces[idx_nearest_face[0]].x

            self.nearest_face = self.list_faces[idx_nearest_face[0]]

            pan_cmd = np.arctan2(self.nearest_face.x, self.nearest_face.z)
            tilt_cmd = -1.0 * np.arctan2(self.nearest_face.y, np.linalg.norm(np.array([self.nearest_face.x, self.nearest_face.z])))
            print "head motion cmd [pan, tilt] is [", np.degrees(pan_cmd), ", ", np.degrees(tilt_cmd), "] degree"

        # if(rospy.Time.now().to_sec() - pointCloudData.header.stamp.to_sec() < 1):
        #     head_pose_to_camera = pointCloudData.points
        #     print head_pose_to_camera

        # movo_head = HeadActionClient()

if __name__ == "__main__":
    print ("start the node")
    rospy.init_node("face_detection")

    # start face tracking
    FaceTracking()



