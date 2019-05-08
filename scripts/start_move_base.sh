#!/usr/bin/env bash
gnome-terminal -x sh -c "roslaunch thesis move_base.launch"
sleep 0.2
xdotool windowminimize $(xdotool getactivewindow)
