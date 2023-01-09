#!/bin/bash

# MOVO2 is ROS master
#   Interface: is br0
#   IP address is 10.66.171.1


# MOVO1 is ROS slave
#   Interface: is eth0
#   IP address is 10.66.171.2 
#       (prototype systems may use 10.66.171.4; this should be changed in /etc/network/interfaces on MOVO1)


# A remote PC is ROS slave, the default interface is lo (change it to whatever physical interface is connected to movo if working with real hardware), 
#    and the IP address is DHCP from MOVO1

if [ "$HOSTNAME" = movo1 ]; then
    export ROBOT_NETWORK=enp0s25
    export ROS_IP=$(ip -4 address show $ROBOT_NETWORK | grep 'inet' | sed 's/.*inet \([0-9\.]\+\).*/\1/' | head -n 1)
    export ROS_MASTER_URI=http://movo2:11311/
elif [ "$HOSTNAME" = movo2 ]; then
    export ROBOT_NETWORK=enp0s25
    export ROS_IP=$(ip -4 address show $ROBOT_NETWORK | grep 'inet' | sed 's/.*inet \([0-9\.]\+\).*/\1/' | head -n 1)
    export ROS_MASTER_URI=http://movo2:11311/
else
    # This should be changed to whatever physical interface is connected to the robot (ie wlan0, eth0, etc..)
    # we will try and find it and if nothing is found then we will default to lo
    robot_iface=$(ifconfig | awk '/129.97.71/ {print $1}' RS="\n\n" | head -n 1)
    if [ ! -z "$robot_iface" ]
    then
        #We found an interface on the robot subnet so lets use this interface
        export ROBOT_NETWORK=$robot_iface
        export ROS_IP=$(ip -4 address show $ROBOT_NETWORK | grep 'inet' | sed 's/.*inet \([0-9\.]\+\).*/\1/' | head -n 1)
        #now lets try to ping movo to see if it should be the master
        ping -q -c 1 -W 1 movo2 >/dev/null
        temp=$?
        if [ $temp -ne 0 ]; then
            #Could not find movo2 even though we are on the right network just make this PC master
            export ROS_MASTER_URI=http://$ROS_IP:11311/
        else
            #Found movo2 so lets use movo2 and the master_uri 
            export ROS_MASTER_URI=http://movo2:11311/
        fi
    else
        echo "No interface on the movo network, def sim settings"
        #No interface on the movo network; default simulation settings
        export ROBOT_NETWORK=lo
        export ROS_IP=$(ip -4 address show $ROBOT_NETWORK | grep 'inet' | sed 's/.*inet \([0-9\.]\+\).*/\1/')
        export ROS_MASTER_URI=http://$ROS_IP:11311/            
    fi
fi

