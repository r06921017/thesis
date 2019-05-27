#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
import time
import qi
import os
import datetime
import subprocess
from math import atan2
from numpy.linalg import norm

import actionlib
import dynamic_reconfigure.client
import rosnode
from actionlib import SimpleActionClient
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import pickle


# Start move_base
if '/move_base' not in rosnode.get_node_names():
    rospy.loginfo('Start move_base.')
    subprocess.call('~/catkin_ws/src/thesis/scripts/start_move_base.sh', shell=True)
    time.sleep(1.5)

rospy.wait_for_service('/move_base/make_plan', timeout=30)
get_global_path = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
               'SUCCEEDED', 'ABORTED', 'REJECTED',
               'PREEMPTING', 'RECALLING', 'RECALLED',
               'LOST']

office_x, office_y, office_yaw = 0.716, -0.824, 1.927  # location: 0
bedroom_x, bedroom_y, bedroom_yaw = 4.971, -0.005, 2.026  # 1
charge_x, charge_y, charge_yaw = 5.024, -0.318, -2.935  # 2
alley1_x, alley1_y, alley1_yaw = 3.827, -1.191, -1.164  # 3
alley2_x, alley2_y, alley2_yaw = 5.140, -3.713, 2.017  # 4
living_x, living_y, living_yaw = 3.397, -5.461, 1.927  # 5
dining_x, dining_y, dining_yaw = 6.258, -3.560, 1.353  # 6
greet_x, greet_y, greet_yaw = 2.959, -5.039, -0.491  # 7, greeting for welcome
emer_x, emer_y, emer_yaw = 5.294, -3.869, -1.165  # 8, emergency

loc_symbol = {0: 'office',
              1: 'bedroom',
              2: 'charge',
              3: 'alley1',
              4: 'alley2',
              5: 'livingroom',
              6: 'diningroom',
              7: 'greet',
              8: 'emergency'}

loc = [[office_x, office_y, office_yaw],
       [bedroom_x, bedroom_y, bedroom_yaw],
       [charge_x, charge_y, charge_yaw],
       [alley1_x, alley1_y, alley1_yaw],
       [alley2_x, alley2_y, alley2_yaw],
       [living_x, living_y, living_yaw],
       [dining_x, dining_y, dining_yaw],
       [greet_x, greet_y, greet_yaw],
       [emer_x, emer_y, emer_yaw]]

# Naoqi setting
if rospy.has_param("Pepper_ip"):
    pepper_ip = rospy.get_param("Pepper_ip")
else:
    print 'Pepper_ip is not given'
    pepper_ip = '192.168.0.152'
print 'Pepper_ip = ', pepper_ip

session = qi.Session()

try:
    session.connect("tcp://" + pepper_ip + ":" + str(9559))
except RuntimeError:
    print("tcp://" + pepper_ip + "\"on port" + str(9559) + ".")
    print("Please check your script arguments. Run with -h option for help.")
    sys.exit(1)

motion_service = session.service("ALMotion")
posture_service = session.service("ALRobotPosture")
tts_service = session.service('ALTextToSpeech')
tts_service.setLanguage('English')
asr_service = session.service("ALAnimatedSpeech")
tablet_service = session.service("ALTabletService")
tablet_service.enableWifi()  # ensure that the tablet wifi is enable

# End Naoqi setting


def remind_obj(in_obj):
    if 'obj.pkl' in os.listdir('../config'):
        obj = pickle.load('obj.pkl')

    else:
        office_obj = {'tvmonitor', 'book', 'chair', 'laptop', 'mouse', 'cell phone'}
        bedroom_obj = {'bed'}
        livingroom_obj = {'sofa', 'book', 'pottedplant', 'tvmonitor', 'remote'}
        diningroom_obj = {'diningtable', 'toaster', 'fork', 'bowl', 'spoon', 'knife', 'cup', 'bowl', 'chair'}

        obj = {'office': office_obj, 'bedroom': bedroom_obj, 'livingroom': livingroom_obj, 'diningroom': diningroom_obj}
        pickle.dump(obj, '../config/obj.pkl')  # save obj as pickle file

    out_loc = []
    for key in obj.keys():
        if in_obj in obj[key]:
            out_loc.append(key)

    return out_loc


def change_local_costmap_radius(inflation_radius):
    client = dynamic_reconfigure.client.Client('/move_base/local_costmap/inflation')
    param = {
        'inflate_unknown': False, 'cost_scaling_factor': 10.0, 'inflation_radius': inflation_radius,
        'enabled': True,
        'groups': {
            'cost_scaling_factor': 10.0, 'parent': 0, 'inflation_radius': inflation_radius,
            'groups': {}, 'id': 0, 'name': 'Default', 'parameters': {}, 'enabled': True, 'state': True,
            'inflate_unknown': False, 'type': ''
        }
    }

    client.update_configuration(param)
    return


def get_cur_pos(pos_topic, t):
    """
    Get robot position from amcl
    :param pos_topic: for amcl_pose (str)
    :param t: time out (int)
    :return: x, y, yaw
    """
    try:
        temp_pos = rospy.wait_for_message(pos_topic, PoseWithCovarianceStamped, timeout=t)  # current robot position
        temp_x = temp_pos.pose.pose.position.x
        temp_y = temp_pos.pose.pose.position.y

        temp_yaw = euler_from_quaternion([temp_pos.pose.pose.orientation.x,
                                          temp_pos.pose.pose.orientation.y,
                                          temp_pos.pose.pose.orientation.z,
                                          temp_pos.pose.pose.orientation.w])[2]
        return temp_x, temp_y, temp_yaw
    except rospy.ROSInterruptException as err:
        rospy.logerr('Failed in get_cur_pos: {0}'.format(str(err)))
        exit(1)


def simple_move_base(sac, dest_x, dest_y, dest_yaw, inflation_radius=1.0, with_rotation_first=True):
    print 'inflation_radius = ', inflation_radius

    # robot is static
    cur_x, cur_y, cur_yaw = get_cur_pos('amcl_pose', 1)
    print 'cur_x, cur_y, cur_yaw = ', cur_x, cur_y, cur_yaw

    if not isin_dest(cur_x, cur_y, cur_yaw, dest_x, dest_y, dest_yaw):

        start = PoseStamped()
        start.header.frame_id = 'map'
        start.pose.position.x = cur_x
        start.pose.position.y = cur_y

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = dest_x
        goal.pose.position.y = dest_y

        # # noinspection PyBroadException
        # try:
        #     subprocess.call('~/catkin_ws/src/thesis/scripts/stop_move_base.sh', shell=True)
        # except Exception:
        #     pass
        #
        # subprocess.call('~/catkin_ws/src/thesis/scripts/start_move_base.sh', shell=True)
        # time.sleep(1.5)

        if '/move_base' in rosnode.get_node_names():
            rospy.loginfo('move_base node succeed!')

            # Simple Action Client
            # sac = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # type: SimpleActionClient

            change_local_costmap_radius(inflation_radius)

            if with_rotation_first:
                temp_global_path = get_global_path(start, goal, 0.1)
                next_location = [temp_global_path.plan.poses[4].pose.position.x,
                                 temp_global_path.plan.poses[4].pose.position.y]

                theta = atan2(next_location[0] - cur_x, next_location[1] - cur_y)  # the angle to path direction
                motion_service.moveTo(0.0, 0.0, (theta - cur_yaw + 0.20), 1)
                posture_service.goToPosture("StandInit", 0.5)
                print 'rotation finish'

            within_time = run_movebase(sac, dest_x, dest_y, dest_yaw)

            if not within_time:
                shutdown(sac)
                rospy.loginfo('Timed out achieving goal.')

            else:
                state = sac.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo('Goal succeed!')

                else:
                    rospy.loginfo('Goal failed with error code:' + str(goal_states[state]))
                    shutdown(sac)
                    inflation_radius -= 0.2
                    simple_move_base(dest_x, dest_y, dest_yaw, inflation_radius, with_rotation_first)
        else:
            # noinspection PyBroadException
            try:
                subprocess.call('~/catkin_ws/src/thesis/scripts/stop_move_base.sh', shell=True)
            except Exception:
                pass
            subprocess.call('~/catkin_ws/src/thesis/scripts/start_move_base.sh', shell=True)
            time.sleep(1.0)

    return


def shutdown(sac):
    rospy.loginfo('Stopping the robot ...')
    sac.cancle_goal()
    rospy.sleep(2)
    cmd_vel_pub.publish(Twist())
    rospy.sleep(1)
    return


def run_movebase(sac, x, y, yaw):
    # Transform euler angle to quaternion
    quat = quaternion_from_euler(0.0, 0.0, yaw)

    # create goal
    goal = MoveBaseGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    sac.wait_for_server()
    sac.send_goal(goal)
    within_time = sac.wait_for_result(rospy.Duration(180))  # 3 mins
    time.sleep(0.5)

    return within_time


def isin_dest(rx, ry, ryaw, des_x, des_y, des_yaw):
    if norm([rx - des_x, ry - des_y]) < 0.5 and abs(float(ryaw - des_yaw)) < 0.3:
        print 'is in destination'
        return True
    else:
        print 'not in destination'
        return False


def set_initial_pose(x, y, yaw, init_location):
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.seq = 1
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp.nsecs = 0

    initial_pose.pose.pose.position.x = x
    initial_pose.pose.pose.position.y = y
    initial_pose.pose.pose.position.z = 0.0

    (_, _, z, w) = quaternion_from_euler(0, 0, yaw)
    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = z
    initial_pose.pose.pose.orientation.w = w

    for i in range(10):
        initial_pose_pub.publish(initial_pose)
        time.sleep(0.5)

    rospy.set_param('/thesis/pepper_location', init_location)

    print 'initial pose sent'
    return
