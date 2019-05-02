#Movo_class
#Author: Nader Mansouri, the AMAZING intern

This package was created as part of an intern's internship and work on Movo.

movo_class/src:

	Cartesian_control.py : This node is used to control the arm in cartesian position using the moveIt Group commander object.
	
	movo_class.py: This class takes root in the moveit_ Move Group Python Interface. It serves as a small API for Movo. This code is useful because someone can plan trajectories and movements with only a python code. 
	
		The link [https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html]. 
		
		It has been adapted for MOVO, or at least, it should work on MOVO. It contains cartesian control and angular position control. 

	face_ICRA.py: This file is used for the ICRA 2019 demo. The code (with face_tracking) make Movo say hello and wave when someone looks directly at it. 

	ICRA_demo.py: This demo lets you launch different demos using the Xbox controller Joystick. It is a demo used for ICRA to have a wireless way to change between demos.

	JC_demo.py and JC_demo_loop.py: This demo is used to demonstrate Movo's capabilities. Is a slightly modified version of the demo_show_basic in movo_demos.

	movo_moveit_test.py and nader_moveit.py: Used as test mediums.

Voice navigation demo:

	voice_input.py: This node is used to verify the speech input from the microphone and recognize it. It verifies if the voice said an existing command. If it did, it will publish on another topic.

	voice_commands.py: This node is used to control Movo with voice commands. It is different from voice_control because this one uses the google_recognizer. It is an online google API so Movo needs to be connected to the internet. The microphone should be selected prior to using the demo for it to work. (Just click on the microphone in Sound settings.)

	voice_nav.py: This node subscribe to the voice_input node. It uses the voice_input and sends a command in the /move_base_simple/goal. This sends a goal and the Movo will move to the goal's location.

movo_class/launch:

	dance_invitation_ICRA.launch: This file is the launch file to launch the dance invitation demo. The song doesn't play and the Movo doesn't wait for an answer for the demo to be launched.
	
	face_ICRA.launch: This file launches the face_tracking demo and the ICRA demo for face_recognition. For each face recognized, the Movo will wave hello (with a certain delay between the hellos)

	ICRA_demo.launch: This launch file launches the demo for Icra demo. 

	robot_voice_map_nav.launch: This launches the robot_map_nav.launch demo. This demo launches all the node necessary for navigating with Movo with your voice.

	sim_map_nav.launch: This launches RViz and Gazebo to let you navigate in simulation with your voice.

	

	
	
	

	
 

	
		
