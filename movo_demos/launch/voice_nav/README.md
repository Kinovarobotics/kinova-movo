Voice navigation demo:

The voice navigation demo uses the move_base node, a map and a microphone to command Movo to a certain position in said map. 

Prior to using the demo, you should map the environment in which Movo will navigate. Make sure the map is accurate and up to date to said location.
Also, when mapping with Movo, it may drift and create some artifacts. These can be smoothened using Paint or Gimp to make a better, cleaner map. Bear in mind
that if the map is too innacurate, the demo will not work. Also, keep in mind that Movo detects from its base only. A table must be drawn in gimp for Movo to avoid
it in real life.

You should also install SpeechRecognition: 'pip install SpeechRecognition'
Also, you need to have the PyAudio 0.2.11+ version for it to work 'pip install PyAudio' 

If the installation doesn't work properly, refer to this site: (GoogleSpeechAPI)[https://pythonspot.com/speech-recognition-using-google-speech-api/]
The demos is located in two folders : /movo_demos/scripts and /movo_demos/launch

	/movo_demos/scripts:
		This folder contains:
					voice_nav.py:	This node subscribes to voice_input. It contains a list of tuples named _goals_. These tuples are: <"NameOfLocation", position in map>
							These "goals" must be modified to fit the map and the command you want to follow. For example, in the associated map,
							Ralph's office should be at x,y,z,w position. w corresponds to orientation. This node, after receiving a text command corresponding to
							a location name of the goal vector publishes in /move_base_simple/goal which will send a command for the robot to go to the location on the map.
					voice_commands.py: These are basic commands like stopping, moving left, right, opening the grippers, responding to thank you,etc.
							This node subscribes to voice_input and publishes on different topic, like /cmd_vel.
					voice_input.py: This node listens using a microphone and use the google_recognizer API to translate voice into text.
							It also verifies if the speech that is heard is recognized as commands for the robot and if it is, publish
							it on a topic that voice_commands and voice_nav subscribe to. If you want to add commands, you should first 
							add them in this file, in the list containing all available commands. 
	/movo_demos/launch:
		This folder contains the necessary launch file to launch the voice_navigation demo. Your map should be located in /movo_demos/maps.
			Both launch files launch Rviz and the 3 python files for the demo to work. For it to work with the map
			of your choosing, you must replace the map_file parameter in the launch file with the name of your map (without the .yaml extension). You
			should also go in [/movo_demos/launch/sim/sim_map_nav.launch]  or [/movo_demos/launch/robot/robot_map_nav.launch] to 
			modify the name of the map_file parameter to match your map.
			
			robot_voice_navigation.launch: This will launch Rviz and the default microphone will be Movo's internal one, which doesn't work well. To change
			it, go in sound settings, and press the appropriate input. I usually do it with a bluetooth one. Also, the demo will only work when Movo
			is connected to the internet (Voice recognition uses an online API). To achieve such a task, I connect my phone to Movo's USB port and use USB tethering 
			to share my connection with Movo.
			
			sim_voice_navigation.launch: If you are in simulation, this will also launch gazebo which will simulate the robot and the laser sensors input.
			If you want the simulation to have appropriate laser sensing, then you should create a gazebo model of your map.
Starting the demo:
	When you start the demo:
	cd ~/movo_ws
	source devel/setup.bash
	roslaunch movo_demos robot_voice_navigation.launch

	Rviz will open on Movo's computer. If Rviz doesn't open on your laptop, you should connect an HDMI cable to Movo2 and use TeamViewer to set the initial position.
	In RViz, using the Set2dPosition button, click on the estimated position of Movo in the Map. The robot will move to that position(in the map) and 
	make a 180degree turn(in real life). This will locate Movo appropriately in the map. After that, using the microphone, command it to go anywhere. 
	If you want Movo to go somewhere, start your sentence with "Go to".
		For example: If you say :"Go to the Boss's office", it should go the the Boss's office. But if you say, "Boss's office", it won't move.
	If you command to positions one after the other, it will change the goal. If you say stop (in emergency cases for example), Movo will stop moving
	for 10 seconds.


