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

```
save_map _mapfile_name_
```
Saves the mapfile on the robot and onboard PC


