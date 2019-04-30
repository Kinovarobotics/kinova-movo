Movo Built in API:

Movo uses ROS to control its components. With an arm and only an arm, the commands would go to the base and there would be inverse kinematics computations to send a position command. 

In ROS, there is a small API, but everything is customizable with ROS. MoveIt offers certain capabilities to Move the arms and you need to adapt it for the Movo yourself.

Here is a little document explaining what some of the files can do, you can always explore what the classes do:


movo_common/movo_ros/src/movo_action_clients:
	move_base_action_client.py is an interface to drive Movo with a certain speed during a certain time.
		_motion_vel: You can send velocity commands to the base. For example, if you send [v_x,v_y, v_z]=[1,2,3], you are sending a command of 1 m/s in x, a command of 2 m/s in y, and a rotation command of 3degrees/second.
		_motion_dist: Instead of sending velocity commands, you can send distance commands. You tell the base to move in x, y, z(rotation). Basically, you are sending them to a relative position from its current positions. If you want, you can record the initial position and move the base in regard to the initial position.
	head_action_client.py is an interface to move the Kinect.
		add_point: this functions takes two angular positions. Those positions corresponds to the pan_joint and tilt_joint. They also take a time (in second). The time corresponds to the start time to go to that trajectory. For example,
				obj=HeadJTASTest()
				obj.add_point(list(current_angles), 0.0)
				obj.add_point([-0.5,1],5)
				obj.start()
		In the code, they use the velocity to compute the time it takes to go to each position. You can also do the same.
	jaco_action_client.py is an interface to move the arms in angular control.
		add_point: This method adds an angular position and a time that corresponds to the time from start to be at that position. You can then create trajectories for the arms with a number of positions to go through.
	torso_action_client.py is an interface to move the torso up and down.
		add_point: This method gives a linear position for the torso and a time point for when it will be.
	gripper_action_client.py is an interface to open and close the grippers.
		command: Commands a position for the grippers. Position 0 corresponds to open position and position 0.165 corresponds to closed position.


movo_common/si_utils/src/si_utils/: This folder contains example codes on how to use these classes for your needs.

-------------------------------------------------------------------------------------------------------
movo_demos:
	There are 2 sets of launch files, one for simulation (replace XXX by sim) and one for the robot when connected(replace XXX by robot) 
	XXX = robot [When you connect to the robot, ROS MASTER is Movo2] and sim [In simulation, ROS_MASTER is your computer]
	XXX_map_nav.launch: Opens Rviz and Gazebo to navigate in a map. The map should be added in movo_demos/maps and the XXX_map_nav.launch file. 
		To use the demo, you click on 2D pose estimate and click on the start point of the map. This will be the initial pose of the robot. 
		Then, you click on 2D Nav goal to set a goal for the robot. The robot will then naviguate to that position using the map and the sensors of the base. 
	XXX_mapping.launch: Launch the mapping node. It opens Rviz for you to visualize how the mapping is going. It also launches the teleop node for you to move the robot around and map the surrounding of the robot. The mapping will update on Rviz and you will be able to save it at the end by running this command line : rosrun map_server map_saver -f nameOfFile (This will save the .pgm and .yaml file in your home directory)

	XXX_robot_demo.launch: Launch a simple demo to showcase the robot capabilities.
	XXX_teleop.launch: Launch the joystick teleop node. You will be able to move the robot around with the joystick connected to the computer or directly with the Movo.
	XXX_assisted_teleop.launch: Launches the teleop.launch node and uses a footprint area around the robot to avoid collision with surrounding objects.
	dance_invitation.launch: Movo will track your face and when it recognizes a face it will position itself and ask the user if it wants to dance. If it does, it will extend its right arm. You will 		be able to control Movo by applying forces on the end effector and make it move.
	face_tracking: The face_tracking will launch and the kinect will follow a person's face (after recognizing it). If you move to fast, the head may not follow you.
	tuck_robot.launch: Will tuck the robot. 
-----------------------------------------------------------------------------------------------------------------

MoveIt interfaces:
	
	To control the robot in cartesian positions, you need to go through MoveIt. [http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html] Two files now exist (movo_demos/scripts/cartesian_control.py and movo_demos/scripts/movo_class.py)

	MoveItCommander interface allows you to control the arms of the Movo individually. To use this class, you need to instantiate a robotCommander. You will be also need to instantiate a scene, and a moveGroupCommander. The example on GitHub lets you modify it to adapt it to your robot.

[https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py]


	
