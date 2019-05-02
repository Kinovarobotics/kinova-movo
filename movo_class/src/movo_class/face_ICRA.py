#!/usr/bin/env python
"""
This code is a demo that needs to be launched with the face_tracking.launch already running.
It recognizes a face, says hello and wave.
"""
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
from datetime import datetime
from std_msgs.msg import String
from movo_action_clients.jaco_action_client import JacoActionClient
from movo_msgs.msg import FaceFound
from movo_msgs.msg import PanTiltCmd
from movo.movo_voice import MovoVoice
from movo_class import Movo_movement
def say(_pub, _data):
    voice_cmd = String()
    voice_cmd.data = _data
    _pub.publish(voice_cmd)

class Face_ICRA:
    def __init__(self):
        rospy.init_node('face_icra')
        rospy.Subscriber("face_detector/face_ID",String, self.face_received)
        rospy.Subscriber("/face_detector/nearest_face", FaceFound, self.face_review)
        self.Publisher=rospy.Publisher("/movo/voice/text", String, queue_size=1, latch=True)
        
        self.found_new_face_0=False
        self.found_new_face_1=False
        
        self.movo_voice = MovoVoice()
        self.movo_larm = Movo_movement('left_arm')
        self.movo_rarm = Movo_movement('right_arm')
        self.larm_greet_deg = [2.18, -0.52, 0.79, 2.09, -1.02, -2.97]
        self.larm_tuck_deg = [1.57, 1.4, 2.5, 2.09, 0, 0]
        
        self.current_index=-1
        
        self.timeOfDay=0
        self.timeSinceLastHello=rospy.get_rostime().secs
    def face_received(self,face_ID):
        rospy.loginfo(face_ID.data)
        self.timeOfDay=datetime.now().time().hour
        #print self.timeOfDay
        b=rospy.get_rostime().secs-self.timeSinceLastHello
        print b
        """
        The next if statement verifies that the face_ID is superior to the last one and that the Movo is currently seeing a face and if it
        has been at least twenty seconds before another hello.
        """
        if((self.current_index < int(face_ID.data) and self.found_new_face_0==True and b>20)):
            self.timeSinceLastHello=rospy.get_rostime().secs
            self.current_index= int(face_ID.data)
            print "Found_face"
            if(0<=self.timeOfDay<11):
                say(self.Publisher,"Good morning.")
            elif(11<=self.timeOfDay<13):
                say(self.Publisher,"Good day. ")
            elif(13 <=self.timeOfDay<17):
                say(self.Publisher,"Good afternoon.")
            elif(17<=self.timeOfDay<24):
                say(self.TimeOfDay,"Good evening.")
            self.greet()
            rospy.sleep(25)
            print("done")    
        self.found_new_face_0=False
    def face_review(self,face):
        #rospy.loginfo(face.header)
        self.found_new_face_0=True
    def greet(self):
        #The arms will wave hello. It uses the movo_class API.
        self.movo_larm.go_to_joint_state([1.57, 1.4, 2.5, 1.09, 0, 0])
        self.movo_larm.go_to_joint_state([1.9, 0, 1.5, 1.09, 0, 0])
        self.movo_larm.go_to_joint_state([1.1, 0, 1.5, 1.09, 0, 0])
        self.movo_larm.go_to_joint_state([1.9, 0, 1.5, 1.09, 0, 0])
        self.movo_larm.go_to_joint_state([1.1, 0, 1.5, 1.09, 0, 0])
        self.movo_larm.go_to_joint_state([1.9, 0, 1.5, 1.09, 0, 0])
        self.movo_larm.go_to_joint_state([1.57, 1.4, 2.5, 1.09, 0, 0])
if __name__ == "__main__":
    # start face tracking
    Face_ICRA()
    rospy.spin()
