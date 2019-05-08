#!/usr/bin/env bash
export Pepper_ip="192.168.0.152";
#export Pepper_ip="192.168.0.184";

gnome-terminal --tab -x sh -c 'roslaunch thesis demo.launch;exec bash'
gnome-terminal --tab -x sh -c 'sh ~/catkin_ws/src/thesis/pepper_driver.sh;exec bash'

#  Deactivate Pepper's self obstacle avoidance
#  gnome-terminal -x sh -c "rosrun pepper_try deactivate_obstacle_avoidance.py"
