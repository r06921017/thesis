#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import subprocess
import time
from numpy.linalg import norm
import numpy as np

import rosnode
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from tf.transformations import quaternion_from_euler


def get_path_step(start_num, dest_num):
    print 'start: ', loc[start_num][0], loc[start_num][1], loc[start_num][2]
    print 'dest: ', loc[dest_num][0], loc[dest_num][1], loc[dest_num][2]

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
    print type(temp_global_path)
    print len(temp_global_path.plan.poses)

    print 'first pose = '
    print temp_global_path.plan.poses[0].pose

    print 'end pose = '
    print temp_global_path.plan.poses[-1].pose

    #
    # temp_x = temp_global_path.plan.poses[0].pose.position.x
    # temp_y = temp_global_path.plan.poses[0].pose.position.y
    #
    # total_dis = norm(np.array([temp_x - loc[start_num][0], temp_y - loc[start_num][1]]))
    # for i in range(len(temp_global_path.plan.poses)):
    #     x1 = temp_global_path.plan.poses[i].pose.position.x
    #     y1 = temp_global_path.plan.poses[i].pose.position.y
    #
    #     total_dis +=

    return


if __name__ == '__main__':
    office_x, office_y, office_yaw = 0.716, -0.824, 1.927  # location: 0
    bedroom_x, bedroom_y, bedroom_yaw = 4.971, -0.005, 2.026  # 1
    charge_x, charge_y, charge_yaw = 5.024, -0.318, -2.935
    alley_x, alley_y, alley_yaw = 5.140, -3.713, 2.017
    living_x, living_y, living_yaw = 3.397, -5.461, 1.927
    dining_x, dining_y, dining_yaw = 6.258, -3.560, 1.353
    emer_x, emer_y, emer_yaw = 5.294, -3.869, -1.165  # emergency

    loc_symbol = ['office', 'bedroom', 'charge', 'alley', 'living room', 'dining room']

    loc = [[office_x, office_y, office_yaw],
           [bedroom_x, bedroom_y, bedroom_yaw],
           [charge_x, charge_y, charge_yaw],
           [alley_x, alley_y, alley_yaw],
           [living_x, living_y, living_yaw],
           [dining_x, dining_y, dining_yaw],
           [emer_x, emer_y, emer_yaw]]

    if '/move_base' not in rosnode.get_node_names():
        rospy.loginfo('Start move_base in simulation.')
        subprocess.call('~/catkin_ws/src/pepper_try/scripts/start_move_base.sh', shell=True)
        time.sleep(3)

    rospy.wait_for_service('/move_base/make_plan', timeout=30)
    get_global_path = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

    get_path_step(2, 3)
