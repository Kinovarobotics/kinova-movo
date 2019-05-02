#!/usr/bin/env python
import speech_recognition as sr
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PointStamped, PoseWithCovarianceStamped, Point, Quaternion, Twist
from std_msgs.msg import String
#These are the names of the goals and commands that are recognized as real commands. These keywords, if heard
#will cause a publish to happen.
locations = ["boss", "friend", "kitchen", "home", "lobby","hallway"]
commands= ["stop", "open", "close", "thank", "hi","start", "left", "right", "forward", "back"]

class voiceInput(object):
	def __init__(self):
		self.heard= rospy.Publisher('/Microphone/speech_recognition/', String,queue_size=10)
		self.text = ""
	def listen(self):
		r =sr.Recognizer()
		with sr.Microphone() as source: #sr.Microphone(6) pour Movo
			print("Say something")
			audio =r.listen(source,phrase_time_limit=5) #Will listen to the microphone for 5 seconds.
		try:
			response= r.recognize_google(audio) # or r.recognize_sphinx(audio)
			print response
			try:
				output_pub=self.verify(response)
				if output_pub != "":
					print output_pub
					self.heard.publish(output_pub)
			except:
				print("publisher failed") 
		except sr.UnknownValueError:
			print("Google Speech Recognition could not understand audio")
		except sr.RequestError as e:
			print("Could not request results from Google Speech Recognition service; {0}".format(e))		   
	def verify(self, r_input):
		#For a location, if the keyword for a location is heard with go to, it will return the location name.
		if "go to" in r_input.lower():
			for i in locations:
				if i in r_input.lower():
					return i
		elif r_input.lower()!= "":
			for i in commands:
				if i in r_input.lower():
					return i
		else:
			return ""
			





if __name__=="__main__":
	rospy.init_node('voice_input')
	v_input = voiceInput()
	while True:
		v_input.listen()
