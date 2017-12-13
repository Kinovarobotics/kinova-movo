@#
@# Author: Mike Purvis <mpurvis@clearpathrobotics.com>
@#         Copyright (c) 2013-2014, Clearpath Robotics, Inc.
@#
@# Redistribution and use in source and binary forms, with or without
@# modification, are permitted provided that the following conditions are met:
@#    * Redistributions of source code must retain the above copyright
@#       notice, this list of conditions and the following disclaimer.
@#    * Redistributions in binary form must reproduce the above copyright
@#       notice, this list of conditions and the following disclaimer in the
@#       documentation and/or other materials provided with the distribution.
@#    * Neither the name of Clearpath Robotics, Inc. nor the
@#       names of its contributors may be used to endorse or promote products
@#       derived from this software without specific prior written permission.
@#
@# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
@# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
@# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
@# DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
@# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
@# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
@# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
@# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
@# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
@# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
@#
@# Please send comments, questions, or patches to code@clearpathrobotics.com
#!/bin/bash
# THIS IS A GENERATED FILE, NOT RECOMMENDED TO EDIT.
# THE TEMPLATE HAS BEEN MODIFIED SPECIFICALLY FOR MOVO

function log() {
  logger -s -p user.$1 ${@@:2}
}

log info "@(name): Using workspace setup file @(workspace_setup)"
source @(workspace_setup)
JOB_FOLDER=@(job_path)

log_path="@(log_path)"
if [[ ! -d $log_path ]]; then
  CREATED_LOGDIR=true
  trap 'CREATED_LOGDIR=false' ERR
    log warn "@(name): The log directory you specified \"$log_path\" does not exist. Attempting to create."
    mkdir -p $log_path 2>/dev/null
    chown {user}:{user} $log_path 2>/dev/null
    chmod ug+wr $log_path 2>/dev/null
  trap - ERR
  # if log_path could not be created, default to tmp
  if [[ $CREATED_LOGDIR == false ]]; then
    log warn "@(name): The log directory you specified \"$log_path\" cannot be created. Defaulting to \"/tmp\"!"
    log_path="/tmp"
  fi
fi

@[if interface]@
export ROS_IP=`rosrun movo_upstart getifip @(interface)`
if [ "$ROS_IP" = "" ]; then
  log err "@(name): No IP address on @(interface), cannot roslaunch."
  exit 1
fi
@[else]@
export ROS_HOSTNAME=$(hostname)
@[end if]@

@[if master_uri]@
export ROS_MASTER_URI=@(master_uri)
@[else]@
export ROS_MASTER_URI=http://127.0.0.1:11311
@[end if]@

log info "@(name): Launching ROS_HOSTNAME=$ROS_HOSTNAME, ROS_IP=$ROS_IP, ROS_MASTER_URI=$ROS_MASTER_URI, ROS_LOG_DIR=$log_path"

# Assemble amalgamated launchfile.
LAUNCH_FILENAME=$log_path/@(name).launch
rosrun movo_upstart mklaunch $JOB_FOLDER > $LAUNCH_FILENAME
if [[ "$?" != "0" ]]; then
  log err "@(name): Unable to generate amalgamated launchfile."
  exit 1
fi
log info "@(name): Generated launchfile: $LAUNCH_FILENAME"

# Warn and exit if setuidgid is missing from the system.
which setuidgid > /dev/null
if [ "$?" != "0" ]; then
  log err "@(name): Can't launch as unprivileged user without setuidgid. Please install daemontools package."
  exit 1
fi

temp=1
cnts=0
while [ $cnts -lt 10 ] && [ $temp -ne 0 ]; do
  ping -q -c 1 -W 1 movo2 >/dev/null
  temp=$?
  cnts=$[$cnts+1]
done 
if [ $temp -ne 0 ]; then
  echo "Unable to ping movo2."
  exit 1
else
  echo "Successfully pinged movo2." 
fi

temp=1
cnts=0
while [ $cnts -lt 10 ] && [ $temp -ne 0 ]; do
  ping -q -c 1 -W 1 "$MOVO_IP_ADDRESS" >/dev/null
  temp=$?
  cnts=$[$cnts+1]
done  

if [ $temp -ne 0 ]; then
  echo "Unable to ping platform at $MOVO_IP_ADDRESS."
  exit 1
else
  echo "Successfully pinged Movo platform at $MOVO_IP_ADDRESS." 
fi

temp=1
cnts=0
while [ $cnts -lt 10 ] && [ $temp -ne 0 ]; do
  ping -q -c 1 -W 1 "$KINOVA_LEFT_ARM_IP_ADDRESS" >/dev/null
  temp=$?
  cnts=$[$cnts+1]
done  

if [ $temp -ne 0 ]; then
  echo "Unable to ping left arm at $KINOVA_LEFT_ARM_IP_ADDRESS."
  exit 1
else
  echo "Successfully pinged Movo left arm at $KINOVA_LEFT_ARM_IP_ADDRESS." 
fi

temp=1
cnts=0
while [ $cnts -lt 10 ] && [ $temp -ne 0 ]; do
  ping -q -c 1 -W 1 "$KINOVA_RIGHT_ARM_IP_ADDRESS" >/dev/null
  temp=$?
  cnts=$[$cnts+1]
done  

if [ $temp -ne 0 ]; then
  echo "Unable to ping right arm at $KINOVA_RIGHT_ARM_IP_ADDRESS."
  exit 1
else
  echo "Successfully pinged Movo right arm at $KINOVA_RIGHT_ARM_IP_ADDRESS." 
fi

temp=1
cnts=0
while [ $cnts -lt 5 ] && [ $temp -ne 0 ]; do
  ping -q -c 1 -W 1 $MOVO_LASER1_IP >/dev/null
  temp=$?
  cnts=$[$cnts+1]
done

if [ $temp -ne 0 ]; then
  echo "failed to ping LASER1 at $MOVO_LASER1_IP."
  exit 1
else
  echo "Successfully pinged LASER1 at $MOVO_LASER1_IP." 
fi

temp=1
cnts=0
while [ $cnts -lt 5 ] && [ $temp -ne 0 ]; do
  ping -q -c 1 -W 1 $MOVO_LASER2_IP >/dev/null
  temp=$?
  cnts=$[$cnts+1]
done

if [ $temp -ne 0 ]; then
  echo "failed to ping LASER2 at $MOVO_LASER2_IP."
  exit 1
else
  echo "Successfully pinged LASER2 at $MOVO_LASER2_IP." 
fi

echo Delaying 5 seconds....
sleep 5

# Punch it.
export ROS_HOME=$(echo ~@(user))/.ros
export ROS_LOG_DIR=$log_path
setuidgid @(user) roslaunch $LAUNCH_FILENAME @(roslaunch_wait?'--wait ')&
PID=$!

log info "@(name): Started roslaunch as background process, PID $PID, ROS_LOG_DIR=$ROS_LOG_DIR"
echo "$PID" > $log_path/@(name).pid

wait "$PID"
