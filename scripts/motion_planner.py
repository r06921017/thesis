#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Make robot move from node to node.
"""
import rospy
import numpy as np
from numpy.linalg import norm
import qi
from nav_msgs.srv import GetPlan

from human_id import *
from thesis.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def dis2dest(r_pos, des):
    rx = r_pos.pose.pose.position.x
    ry = r_pos.pose.pose.position.y
    ryaw = euler_from_quaternion(r_pos.pose.pose.orientation)[2]

    if


def motion_cb(data):
    """
    Navigation based on instructions
    :param data: InstructionArray
    :return: None
    """
    # move to destination from node to node
    for ins in data:
        r_pos = rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped, timeout=5)  # get current robot position
        des_x, des_y, des_yaw = loc[ins.destination][0], loc[ins.destination][1], loc[ins.destination][2]
        if norm(r_pos - des)

    return


if __name__ == '__main__':
    rospy.init_node('motion_planner', anonymous=True, log_level=rospy.loginfo)

    office_x, office_y, office_yaw      = 0.716, -0.824,  1.927  # location: 0
    bedroom_x, bedroom_y, bedroom_yaw   = 4.971, -0.005,  2.026  # 1
    charge_x, charge_y, charge_yaw      = 5.024, -0.318, -2.935
    alley_x, alley_y, alley_yaw         = 5.140, -3.713,  2.017
    living_x, living_y, living_yaw      = 3.397, -5.461,  1.927
    dining_x, dining_y, dining_yaw      = 6.258, -3.560,  1.353

    loc = [[office_x, office_y, office_yaw],
          [bedroom_x, bedroom_y, bedroom_yaw],
          [charge_x, charge_y, charge_yaw],
          [alley_x, alley_y, alley_yaw],
          [living_x, living_y, living_yaw],
          [dining_x, dining_y, dining_yaw ]]

    get_global_path = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

    rospy.Subscriber('/thesis/instruction_buffer', InstructionArray, motion_cb, queue_size=1)
    rospy.spin()
