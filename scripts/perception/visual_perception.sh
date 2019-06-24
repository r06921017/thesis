#!/usr/bin/env bash

#gnome-terminal --tab -x sh -c "rosparam set /thesis/camera /thesis/img_stitching"
gnome-terminal --tab -x sh -c  "rosrun thesis img_stitching.py"
gnome-terminal --tab -x sh -c "roslaunch thesis yolo_pepper_stitching.launch"
sleep 5
gnome-terminal --tab -x sh -c "rosrun thesis pepper_openpose.py"
sleep 1

# for action recognition
gnome-terminal --tab -x sh -c "rosparam set /thesis/action_on True"
gnome-terminal --tab -x sh -c "rosrun thesis action_recognition.py"
gnome-terminal --tab -x sh -c "rosrun rviz rviz -d /home/robot/.rviz/yolo_pose_viz.rviz"

# for localization
# gnome-terminal --tab -x sh -c "rosrun thesis get_human.py"
