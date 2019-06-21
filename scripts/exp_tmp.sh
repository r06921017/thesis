#!/usr/bin/env bash
END=10

for i in $(seq 1 ${END});
do
    SEED=$((1000 + ${END}))
    echo ${SEED}
    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor.py  --max_num 10 --is_sim 1 --is_rand 1 --seed "${SEED}";exec bash"
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_opt_sim.py;exec bash"
    sleep 400
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_opt_sim"
    sleep 3
    END=$((END+10))
done

for i in $(seq 1 ${END});
do
    SEED=$((1000 + ${END}))
    echo ${SEED}
    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor.py  --max_num 10 --is_sim 1 --is_rand 1 --seed "${SEED}";exec bash"
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_fcfs_sim.py;exec bash"
    sleep 300
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_fcfs_sim"
    sleep 3
    END=$((END+10))
done

for i in $(seq 1 ${END});
do
    SEED=$((1000 + ${END}))
    echo ${SEED}
    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor.py  --max_num 10 --is_sim 1 --is_rand 1 --seed "${SEED}";exec bash"
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_rand_sim.py;exec bash"
    sleep 300
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_rand_sim"
    sleep 3
    END=$((END+10))
done

for i in $(seq 1 ${END});
do
    SEED=$((1000 + ${END}))
    echo ${SEED}
    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor.py  --max_num 10 --is_sim 1 --is_rand 1 --seed "${SEED}";exec bash"
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_pf_sim.py;exec bash"
    sleep 250
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_pf_sim"
    sleep 3
    END=$((END+10))
done

for i in $(seq 1 ${END});
do
    SEED=$((1000 + ${END}))
    echo ${SEED}
    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor.py  --max_num 10 --is_sim 1 --is_rand 1 --seed "${SEED}";exec bash"
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_sf_sim.py;exec bash"
    sleep 250
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_sf_sim"
    sleep 3
    END=$((END+10))
done

for i in $(seq 1 ${END});
do
    SEED=$((1000 + ${END}))
    echo ${SEED}
    gnome-terminal --tab -x sh -c "rosrun thesis instruction_constructor.py  --max_num 10 --is_sim 1 --is_rand 1 --seed "${SEED}";exec bash"
    sleep 2
    gnome-terminal --tab -x sh -c "rosrun thesis task_motion_planner_dp_sim.py;exec bash"
    sleep 250
    gnome-terminal --tab -x sh -c "rosnode kill /task_motion_planner_dp_sim"
    sleep 3
    END=$((END+10))
done
