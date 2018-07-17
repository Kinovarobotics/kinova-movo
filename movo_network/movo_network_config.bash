export ROS_IP=$(gethostip localhost | awk '{print $2}')
export ROS_MASTER_IPADD=$ROS_IP
export ROS_MASTER_URI=http://$ROS_IP:11311/

