#!/usr/bin/env bash
END=10

for i in $(seq 1 ${END});
do
    SEED=$((1000 + ${END}))
    echo ${SEED}
    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor.py  --max_num 10 --is_sim 1 --is_rand 1 --seed "${SEED}
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_opt_sim.py"
    sleep 1000
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_opt_sim"
    gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor"
    sleep 3

    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor.py  --max_num 10 --is_sim 1 --is_rand 1 --seed "${SEED}
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_fcfs_sim.py"
    sleep 400
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_fcfs_sim"
    gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor"
    sleep 3

    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor.py  --max_num 10 --is_sim 1 --is_rand 1 --seed "${SEED}
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_rand_sim.py"
    sleep 400
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_rand_sim"
    gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor"
    sleep 3

    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor.py  --max_num 10 --is_sim 1 --is_rand 1 --seed "${SEED}
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_pf_sim.py"
    sleep 250
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_pf_sim"
    gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor"
    sleep 3

    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor.py  --max_num 10 --is_sim 1 --is_rand 1 --seed "${SEED}
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_sf_sim.py"
    sleep 250
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_sf_sim"
    gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor"
    sleep 3

    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor.py  --max_num 10 --is_sim 1 --is_rand 1 --seed "${SEED}
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_dp_sim.py"
    sleep 250
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_dp_sim"
    gnome-terminal --tab -x sh -c "rosnode kill /instruction_constructor"
    sleep 3

    END=$((END+10))
done
