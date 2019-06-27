#!/usr/bin/env bash

gnome-terminal --tab -x sh -c "rosparam set /thesis/init_tmp false"

gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor_v2.py  --max_num 10 --is_sim 1 --is_rand 0"
sleep 2
gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_opt_sim.py"
sleep 1030
gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_opt_sim"
gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor_v2"
gnome-terminal --tab -x sh -c "rosparam set /thesis/init_tmp false"
sleep 3

gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor_v2.py  --max_num 10 --is_sim 1 --is_rand 0"
sleep 2
gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_fcfs_sim.py"
sleep 530
gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_fcfs_sim"
gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor_v2"
gnome-terminal --tab -x sh -c "rosparam set /thesis/init_tmp false"
sleep 3

gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor_v2.py  --max_num 10 --is_sim 1 --is_rand 0"
sleep 2
gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_rand_sim.py"
sleep 530
gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_rand_sim"
gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor_v2"
gnome-terminal --tab -x sh -c "rosparam set /thesis/init_tmp false"
sleep 3

gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor_v2.py  --max_num 10 --is_sim 1 --is_rand 0"
sleep 2
gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_pf_sim.py"
sleep 530
gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_pf_sim"
gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor_v2"
gnome-terminal --tab -x sh -c "rosparam set /thesis/init_tmp false"
sleep 3

gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor_v2.py  --max_num 10 --is_sim 1 --is_rand 0"
sleep 2
gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_sf_sim.py"
sleep 530
gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_sf_sim"
gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor_v2"
gnome-terminal --tab -x sh -c "rosparam set /thesis/init_tmp false"
sleep 3

gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor_v2.py  --max_num 10 --is_sim 1 --is_rand 0"
sleep 2
gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_dp_sim.py"
sleep 530
gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_dp_sim"
gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor_v2"
gnome-terminal --tab -x sh -c "rosparam set /thesis/init_tmp false"
sleep 3
