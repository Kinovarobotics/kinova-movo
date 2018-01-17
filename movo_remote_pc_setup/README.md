# movo_remote_pc_setup
setup scripts to get a remote PC interfaced with movo

**Please read this carefully! This is provided for convenience only and will only work with a Movo robot. It is intended to be used with a remote PC that has ubuntu 14.04.5LTS cleanly installed. It will configure the system and install all necessary updates to interface with MoVo. This process may change the environment if there is one already setup. Be sure you read through setup_remote_pc in this directory if that is the case, and make sure you fully understand what it is doing. Kinova is not responsible in any way for the use of this script and potential loss of data if it is run on any other configuration.**

## Installation
To install and setup all the components on a remote PC you need to checkout the latest software, the location of the ws matters

* Create a workspace and clone the repo into the src directory
```
mkdir ~/movo_ws && cd ~/movo_ws && git clone https://github.com/Kinovarobotics/kinova-movo.git src/
```
* Run the setup script and follow the prompts
```
cd ~/movo_ws/src/movo_remote_pc_setup && ./setup_remote_pc
```

## Using the installed components
Now that you have run the setup script you can take advantage of convenient functions for running demos and updates on the robot. The section below defines the bash aliases and functions added by the setup script. To run one open a terminal and type the command.

**Important note: To run any of the sim_ functions please disconnect the remote PC from the robot.**

**Important note: Always open a new terminal or tab when running new functions. This ensures that any changes to the environment variables will get applied.**

### Bash commands
This section defines the bash aliases and functions that are added to help users more easily work with Movo

#### Functions for starting demos
```
sim_demo
```
Starts the simulation of the pick and place demo. It takes about 45 seconds to fully launch due to the complexity of the gazebo simulation **(BE PATIENT)**. Once the demo is up and running it is identical to the 
real demo.

##### Demo Outline
* A table is placed ~2m infront of movo
* A standard can of pringles is placed near the edge to the right of center
* A standard 12oz can (beer or soda can) is placed near the edge to the left of center
* Robot initializes into the plan grasop pose
* Robot finds the table and the objects
* Robot uses the navigation stack to navigate to the optimal picking position
* Robot identifies the beer can using PCL segmentation and object primatives
* Robot plans valid grasping poses
* Robot plans the pick operation for the beer can 
* Robot picks up the beer can
* Robot returns to the plan grasp pose
* Robot identifies the pringles can using PCL segmentation and object primatives
* Robot plans valid grasping poses
* Robot plans the pick operation for the pringles can 
* Robot picks up the pringles can
* Robot returns to the plan grasp pose
* Robot plans the place operation for the beer can 
* Robot places the beer can back in its original position
* Robot returns to the plan grasp pose
* Robot plans the place operation for the pringles can 
* Robot places the pringles can back in its original position
* Robot returns to the plan grasp pose
* Robot moves to tucked position
* Robot uses navigation stack to return to original start location
    
```
sim_teleop
```
add text here

```
sim_mapping
```
add text here

```
sim_sensor_nav
```
add text here

```
sim_map_nav
```
add text here

```
sim_assisted_teleop
```
add text here

```
robot_demo
```
Starts the pick and place demo on the actual robot. To get this setup you need:
##### Required items for demo
1. A folding table like the one used to develop the demo, [Lifetime 4' Utility Table](https://www.amazon.com/Lifetime-22950-Folding-Utility-Granite/dp/B0002U3V8Q/ref=sr_1_1_sspa?ie=UTF8&qid=1516207875&sr=8-1-spons&keywords=lifetime+4%27+table&psc=1)
2. A standard 5.68oz pringles can [Prinlges](https://www.pringles.com/us/products/favorites/the-original.html)
3. A standard 12oz soda or beer can [Standard 12oz can dimensions](http://www.cask.com/files/pdf/techspecs/12oz-can-drawing.pdf)
##### Setup notes
1. The table selected should have a light colored top or a light colored table cloth. The Kinect is a TOF IR sensor so is sensitive to very dark or reflective surfaces.
2. The table should be spaced off of the wall ~2ft. The demo determines the size of the table or optimal pick positioning and segmentation works best if table is no right against the wall
3. Cans should be placed within 4-20cm of center with the beer can on the left and pringles can on the right
4. Robot should start facing the table, roughly centered, and ~2m from the center of the table to the center of the robot.

##### Demo Outline    
* A table is placed ~2m infront of movo
* A standard can of pringles is placed near the edge to the right of center
* A standard 12oz can (beer or soda can) is placed near the edge to the left of center
* Robot initializes into the plan grasop pose
* Robot finds the table and the objects
* Robot uses the navigation stack to navigate to the optimal picking position
* Robot identifies the beer can using PCL segmentation and object primatives
* Robot plans valid grasping poses
* Robot plans the pick operation for the beer can 
* Robot picks up the beer can
* Robot returns to the plan grasp pose
* Robot identifies the pringles can using PCL segmentation and object primatives
* Robot plans valid grasping poses
* Robot plans the pick operation for the pringles can 
* Robot picks up the pringles can
* Robot returns to the plan grasp pose
* Robot plans the place operation for the beer can 
* Robot places the beer can back in its original position
* Robot returns to the plan grasp pose
* Robot plans the place operation for the pringles can 
* Robot places the pringles can back in its original position
* Robot returns to the plan grasp pose
* Robot moves to tucked position
* Robot uses navigation stack to return to original start location

```
robot_teleop
```
add text here

```
robot_mapping
```
add text here

```
robot_sensor_nav
```
add text here

```
robot_map_nav filename
```
add text here

```
robot_assisted_teleop
```
add text here


#### Functions for saving maps and updating the robot PCs
```
save_map [filename]
```
Saves the mapfile on the robot and onboard PC. **_filename_** is the user defined mapfile name

```
sync_robot [compile options]
```
Allows user to make changes locally on the remote PC in ~/movo_ws/src and push the changes to the robot
* compile options **must specify one**
    * -nc Do not compile just copy the files from the remote PC workspace to the robot and restart the service on the robot. Useful to save time when just launch files or python files are changed.
    * -c copy the files from the remote PC workspace to the robot, compile, and restart the service

#### Generally Useful Functions
```
m2
```
Alias for SSH into movo2. Equivalent to ssh -X movo@movo2

```
m1
```
Alias for SSH into movo2. Equivalent to ssh -X movo@movo1

```
movostop
```
Stops the upstart service on movo2 that runs the ROS software on the robot at startup

```
movostart
```
Starts the upstart service on movo2 that runs the ROS software on the robot

```
movochk
```
Tails the last 30 lines of the upstart logfile to print out what you would normally see from launching a file

```
sws
```
Sources the workspace from a top level directory (same as source ./devel/setup.sh)

```
clean_backups
```
Removes all the *~ backupfiles from a directory tree. Useful for cleanup

```
clean_pyc
```
Removes all the *.pyc compiled python from a directory tree. Useful for cleanup

```
clean_rosbuild
```
Removes the devel/ and build/ directories from a workspace. Useful for cleanup

```
kill_robot_pcs
```
Shutsdown movo1 and movo2

```
fix_gvfs
```
Run this if you ever get the bash error _cannot access .gvfs: Permission denied_. It is included for convenience

```
killgazebo
```
Kills all processes associated with Gazebo if it hangs at shutdown. It is included for convenience

```
killros
```
Kills all processes associated with ROS if any hangs at shutdown. It is included for convenience

```
fix_stl
```
Fixes an STL file containing the string solid, which ASSIMP does not like

```
fix_perm
```
Fixes permissions on executable files in a ROS workspace. It is useful when copying code from a USB stick where the permissions were not preserved.


