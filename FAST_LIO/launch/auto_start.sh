#!/bin/bash
sleep 5
source /opt/ros/noetic/setup.bash
source /home/rm/ws_livox/devel/setup.bash
source /home/rm/sp_nav/devel/setup.bash
export ROS_PACKAGE_PATH=/home/rm/ws_livox/src:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=/home/rm/sp_nav/src:$ROS_PACKAGE_PATH
gnome-terminal -- bash -c "roslaunch livox_ros_driver2 msg_MID360.launch"& sleep 3s
gnome-terminal -- bash -c "roslaunch fast_lio relocalization_nav.launch  "& sleep 1s
#gnome-terminal -- bash -c "roslaunch fast_lio mapping_mid_360_nav.launch  "& sleep 1s
echo " " | sudo -S ip link set can0 down
echo " " | sudo -S ip link set can0 up type can bitrate 1000000
gnome-terminal -- bash -c "rosrun sentry_communicator sentry_communicator "& sleep 1s
gnome-terminal -- bash -c "roslaunch sp_decision sp_decision.launch  "& sleep 1s
