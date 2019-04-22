#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import subprocess
import time
import numpy as np
from numpy.linalg import norm

import rosnode
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from tf.transformations import quaternion_from_euler


def get_path_step(start_num, dest_num):
    print 'start: {0}, goal: {1}'.format(loc_symbol[start_num], loc_symbol[dest_num])

    # print 'start: ', loc[start_num][0], loc[start_num][1], loc[start_num][2]
    # print 'dest: ', loc[dest_num][0], loc[dest_num][1], loc[dest_num][2]

    start = PoseStamped()
    start.header.frame_id = 'map'
    start.pose.position.x = loc[start_num][0]
    start.pose.position.y = loc[start_num][1]
    start.pose.orientation.z = quaternion_from_euler(0.0, 0.0, loc[start_num][2])[2]
    start.pose.orientation.w = quaternion_from_euler(0.0, 0.0, loc[start_num][2])[3]

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = loc[dest_num][0]
    goal.pose.position.y = loc[dest_num][1]
    goal.pose.orientation.z = quaternion_from_euler(0.0, 0.0, loc[dest_num][2])[2]
    goal.pose.orientation.z = quaternion_from_euler(0.0, 0.0, loc[dest_num][2])[3]

    temp_global_path = get_global_path(start, goal, 0.1)

    temp_x = temp_global_path.plan.poses[0].pose.position.x
    temp_y = temp_global_path.plan.poses[0].pose.position.y

    total_dis = norm([temp_x - loc[start_num][0], temp_y - loc[start_num][1]])
    for i in range(len(temp_global_path.plan.poses) - 1):
        x1 = temp_global_path.plan.poses[i].pose.position.x
        y1 = temp_global_path.plan.poses[i].pose.position.y

        x2 = temp_global_path.plan.poses[i+1].pose.position.x
        y2 = temp_global_path.plan.poses[i+1].pose.position.y

        total_dis += norm([x2-x1, y2-y1])

    total_step = np.ceil(total_dis / (min_vx * step_t))  # type: float

    print 'euclidean dis = ', norm([loc[start_num][0]-loc[dest_num][0], loc[start_num][1]-loc[dest_num][1]])
    print 'total_dis = ', total_dis
    print 'time step in real = ', total_step

    return


if __name__ == '__main__':
    rospy.init_node('get_global_path_len')

    step_t = 2.0
    min_vx = 0.1

    office_x, office_y, office_yaw = 0.716, -0.824, 1.927  # location: 0
    bedroom_x, bedroom_y, bedroom_yaw = 4.971, -0.005, 2.026  # 1
    charge_x, charge_y, charge_yaw = 5.024, -0.318, -2.935  # 2
    alley_x, alley_y, alley_yaw = 5.140, -3.713, 2.017  # 3
    living_x, living_y, living_yaw = 3.397, -5.461, 1.927  # 4
    dining_x, dining_y, dining_yaw = 6.258, -3.560, 1.353  # 5
    greet_x, greet_y, greet_yaw = 2.959, -5.039, -0.491  # 6, greeting for welcome
    emer_x, emer_y, emer_yaw = 5.294, -3.869, -1.165  # 7, emergency

    loc_symbol = {0: 'office', 1: 'bedroom', 2: 'charge', 3: 'alley', 4: 'livingroom',
                  5: 'diningroom', 6: 'greet', 7: 'emergency'}

    loc = [[office_x, office_y, office_yaw],
           [bedroom_x, bedroom_y, bedroom_yaw],
           [charge_x, charge_y, charge_yaw],
           [alley_x, alley_y, alley_yaw],
           [living_x, living_y, living_yaw],
           [dining_x, dining_y, dining_yaw],
           [greet_x, greet_y, greet_yaw],
           [emer_x, emer_y, emer_yaw]]

    if '/move_base' not in rosnode.get_node_names():
        rospy.loginfo('Start move_base in simulation.')
        subprocess.call('~/catkin_ws/src/thesis/scripts/start_move_base.sh', shell=True)
        time.sleep(3)

    rospy.loginfo('Wait for make_plan service.')
    rospy.wait_for_service('/move_base/make_plan', timeout=30)
    get_global_path = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

    for i in range(len(loc_symbol)):
        for j in range(i+1, len(loc_symbol)):
            get_path_step(i, j)
