#!/bin/bash
sleep 5s

source /home/rm/ws_livox/devel/setup.bash
sleep 0.5s
gnome-terminal -- bash -c "roslaunch livox_ros_driver2 msg_MID360.launch;exec bash"&sleep 3s
source /home/rm/sp_nav/devel/setup.bash

gnome-terminal -- bash -c "roslaunch fast_lio relocalization_nav.launch  "& sleep 1s
echo " " | sudo -S ip link set can0 down
echo " " | sudo -S ip link set can0 up type can bitrate 1000000
gnome-terminal -- bash -c "rosrun sentry_communicator sentry_communicator "& sleep 1s

gnome-terminal -- bash -c "roslaunch sp_decision sp_decision.launch;exec bash"& sleep 1s

gnome-terminal -- bash -c "rosbag record --duration=480 /Team_robot_HP /referee_data /referee_info /robot_shoot /robot_state /rotate_state /sentry/cmd_vel /sentry/decision"

