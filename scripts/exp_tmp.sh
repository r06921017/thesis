#!/usr/bin/env bash
END=5

gnome-terminal --tab -x sh -c "rosparam set /thesis/init_tmp false"

for i in $(seq 0 ${END});
do
    SEED=$((500 + i))
    echo ${SEED}
    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor_v2.py  --max_num 5 --is_sim 1 --is_rand 1 --seed "${SEED}
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_opt_sim.py"
    sleep 1000
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_opt_sim"
    gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor_v2"
    gnome-terminal --tab -x sh -c "rosparam set /thesis/init_tmp false"
    sleep 3

    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor_v2.py  --max_num 5 --is_sim 1 --is_rand 1 --seed "${SEED}
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_fcfs_sim.py"
    sleep 500
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_fcfs_sim"
    gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor_v2"
    gnome-terminal --tab -x sh -c "rosparam set /thesis/init_tmp false"
    sleep 3

    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor_v2.py  --max_num 5 --is_sim 1 --is_rand 1 --seed "${SEED}
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_rand_sim.py"
    sleep 500
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_rand_sim"
    gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor_v2"
    gnome-terminal --tab -x sh -c "rosparam set /thesis/init_tmp false"
    sleep 3

    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor_v2.py  --max_num 5 --is_sim 1 --is_rand 1 --seed "${SEED}
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_pf_sim.py"
    sleep 500
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_pf_sim"
    gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor_v2"
    gnome-terminal --tab -x sh -c "rosparam set /thesis/init_tmp false"
    sleep 3

    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor_v2.py  --max_num 5 --is_sim 1 --is_rand 1 --seed "${SEED}
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_sf_sim.py"
    sleep 500
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_sf_sim"
    gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor_v2"
    gnome-terminal --tab -x sh -c "rosparam set /thesis/init_tmp false"
    sleep 3

    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor_v2.py  --max_num 5 --is_sim 1 --is_rand 1 --seed "${SEED}
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_dp_sim.py"
    sleep 500
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_dp_sim"
    gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor_v2"
    gnome-terminal --tab -x sh -c "rosparam set /thesis/init_tmp false"
    sleep 3
done
