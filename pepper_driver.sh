#!/usr/bin/env bash
export computer_ip="$( ip route get 8.8.8.8 | awk 'NR==1 {print $NF}' )"
rosrun naoqi_driver naoqi_driver_node --qi-url=tcp://$Pepper_ip:9559 --roscore_ip $computer_ip --network_interface eno1
