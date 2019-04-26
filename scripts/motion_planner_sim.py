#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
Motion Planner for simulation.
"""

from nav_msgs.srv import GetPlan

from human_id import *
from action_recognition import get_action_cat
from thesis.msg import *
import time


def get_function_sim(instr):
    print 'instr id = ', instr.id
    print 'instr function = ', func_symbol[instr.function]
    time.sleep(instr.duration * step_t)

    return


def simple_move_base_sim(dest_symbol):
    """
    time estimation for robot navigation.
    :param dest_symbol:
    :return:
    """
    time.sleep(step_t)
    return


def motion_cb_sim(data):
    # move to destination from node to node
    cur_location = rospy.get_param('/thesis/pepper_location')  # type: int
    for instr in data:
        print 'go from ', loc_symbol[int(cur_location)], ' to ', loc_symbol[int(instr.destination)]

        # if moving from livingroom/diningroom to office/bedroom/charge, then go to alley first
        if cur_location > 3 > instr.destination:
            simple_move_base_sim(loc_symbol[3])

        simple_move_base_sim(loc_symbol[instr.destination])
        rospy.set_param('/thesis/pepper_location', instr.destination)

        get_function_sim(instr)

    return


if __name__ == '__main__':
    rospy.init_node('motion_planner', anonymous=True)

    # global const for action recognition
    pkg_dir = rospkg.RosPack().get_path('thesis')
    config_dir = pkg_dir + '/config/'
    human_info_dir = pkg_dir + '/human_info/'

    human_dict = load_human_info2dict(human_info_dir)
    action_cat = get_action_cat()

    rospy.set_param('/thesis/pepper_location', 2)  # initial robot location to 'charge'

    step_t = 0.1  # time per time step (in second)

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
    func_symbol = {0: 'NOP', 1: 'chat', 2: 'encourage', 3: 'remind object', 4: 'remind schedule',
                   5: 'check human', 6: 'charge', 7: 'play videos', 8: 'play game', 9: 'emergency'}

    goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                   'SUCCEEDED', 'ABORTED', 'REJECTED',
                   'PREEMPTING', 'RECALLING', 'RECALLED',
                   'LOST']

    rospy.wait_for_service('/move_base/make_plan', timeout=30)
    get_global_path = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

    rospy.Subscriber('/thesis/task_buffer', InstructionArray, motion_cb_sim, queue_size=1)

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
    tabletService = session.service("ALTabletService")
    tabletService.enableWifi()  # ensure that the tablet wifi is enable
    # End Naoqi setting

    rospy.loginfo('Start Motion Planner Simulation!')
    rospy.spin()
