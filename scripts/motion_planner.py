#!/usr/bin/env python
# -*- coding: UTF-8 -*-

"""
Make robot move from node to node.
"""

import datetime
import subprocess
from math import atan2

import actionlib
import dynamic_reconfigure.client
import rosnode
from actionlib import SimpleActionClient
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from thesis.msg import *

from human_id import *
from action_recognition import *


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
    temp_pos = rospy.wait_for_message(pos_topic, PoseWithCovarianceStamped, timeout=t)  # current robot position
    temp_x = temp_pos.pose.pose.position.x
    temp_y = temp_pos.pose.pose.position.y
    temp_yaw = euler_from_quaternion(temp_pos.pose.pose.orientation)[2]

    return temp_x, temp_y, temp_yaw


def simple_move_base(dest_x, dest_y, dest_yaw, inflation_radius=1.0, with_rotation_first=True):
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

        # noinspection PyBroadException
        try:
            subprocess.call('~/catkin_ws/src/thesis/scripts/stop_move_base.sh', shell=True)
        except Exception:
            pass

        subprocess.call('~/catkin_ws/src/pepper_try/scripts/start_move_base.sh', shell=True)
        time.sleep(1.5)

        if '/move_base' in rosnode.get_node_names():
            rospy.loginfo('move_base node succeed!')

            # Simple Action Client
            sac = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # type: SimpleActionClient

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
    if norm(rx - des_x, ry - des_y) < 0.5 and abs(float(ryaw - des_yaw)) < 0.3:
        print 'is in destination'
        return True
    else:
        print 'not in destination'
        return False


def get_function(instr):
    start_time = time.time()

    if instr.function == 0:  # NOP
        say_str = 'Hello ' + instr.target + ', what can I do for you?'
        tts_service.say(say_str)

    elif instr.function == 1:  # chat
        say_str = 'Greetings, ' + instr.target + 'how do you do?'
        tts_service.say(say_str)
        voice_msg = rospy.wait_for_message('/Tablet/voice', VoiceMessage)  # type: VoiceMessage
        human = identify_voice(human_dict['ip'], voice_msg)

        if human is None:  # add new person into human_info
            greeting_cb()
        else:
            # human say: Thank you for chatting with me
            tts_service.say('Well, I am glad to help.')

    elif instr.function == 2:  # encourage
        tts_service.say('What happened?')
        voice_msg = rospy.wait_for_message('/Tablet/voice', VoiceMessage)  # type: VoiceMessage
        if 'break' and 'up' in voice_msg.texts[0].split(' '):
            asr_service.say('Cheer up! you deserve a better one.')
            time.sleep(0.5 * step_t)
            tts_service.say('I will be with you whenever you are down.')
            voice_msg = rospy.wait_for_message('/Tablet/voice', VoiceMessage)
            if 'Thank' or 'thank' or 'Thanks' in voice_msg.texts[0].split(' '):
                asr_service.say('No problem, my friend.')
                posture_service('StandInit', 0.6 * step_t)

    elif instr.function == 3:  # remind object
        obj_loc = remind_obj(instr.target)  # list of locations of given objects
        if obj_loc is None and instr.type == 0:  # no locations founds
            temp_str = 'I do not know where ' + instr.target + 'is.'
            tts_service.say(temp_str)

    elif instr.function == 4:  # remind schedule
        target_human = get_human_from_name(name_dict=human_dict['name'], name=instr.target)
        cur_weekday = datetime.datetime.today().weekday()
        cur_hour = datetime.datetime.now().hour
        # cur_min = datetime.datetime.now().minute
        sch_list = []
        for sch in target_human.schedules:
            if sch.start_day < cur_weekday < sch.end_day and cur_hour < sch.start_hour:
                sch_str = sch.activity + 'at ' + sch.start_hour + ', '
                sch_list.append(sch_str)

        if sch_list is None:
            tts_service.say('There are no schedule, my master.')
        else:
            # formulate as sentence for robot
            temp_str = 'You need to '
            for i, sch_str in enumerate(sch_list):
                if i == len(sch_list) - 1:
                    temp_str += 'and '
                temp_str += sch_str

    elif instr.function == 5:  # check human
        rospy.set_param('/thesis/action_on', True)
        time.sleep(5 * step_t)
        target_human = get_human_from_name(name_dict=human_dict['name'], name=instr.target)

        if instr.type == 0:  # request from human
            source_human = get_human_from_name(name_dict=human_dict['name'], name=instr.source)
            simple_move_base(loc[source_human.location][0],
                             loc[source_human.location][1],
                             loc[source_human.location][2])

        temp_str = target_human.name + ' is ' + action_cat[target_human.action]
        tts_service.say(temp_str)

    elif instr.function == 6:  # charge
        tts_service.say('I need to charge, can anyone help me, please?')
        time.sleep(60 * step_t)  # wait to charge 10 seconds

    elif instr.function == 7:  # play videos
        tts_service.say('Would you like some music?')
        tabletService.playVideo("https://www.youtube.com/watch?v=lmNHeu7DB28")
        time.sleep((instr.duration - (time.time() - start_time) - 0.5) * step_t)

    elif instr.function == 8:  # play games
        tts_service.say('Let\'s play a game!')
        time.sleep((instr.duration - (time.time() - start_time) - 0.5) * step_t)

    elif instr.function == 9:  # emergency
        tts_service.say('Emergency. I will call Li-Pu for help.')
        simple_move_base(emer_x, emer_y, emer_yaw)
        say_str = 'Li-Pu, ' + instr.target + 'may need your help. Please be hurry!'
        asr_service.say(say_str)
        simple_move_base(loc[instr.destination][0], loc[instr.destination][1], loc[instr.destination][2])

    if instr.duration - (time.time() - start_time) > 0.:
        time.sleep((instr.duration - (time.time() - start_time)) * step_t)

    return


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


def motion_cb(data):
    """
    Navigation based on instructions
    :param data: InstructionArray
    :return: None
    """
    # move to destination from node to node
    cur_location = rospy.get_param('/thesis/pepper_location')
    for instr in data:
        rospy.loginfo('go from ', loc_symbol[int(cur_location)], ' to ', loc_symbol[int(instr.destination)])

        # if moving from livingroom/diningroom to office/bedroom/charge, then go to alley first
        if cur_location > 3 > instr.destination:
            simple_move_base(loc[3][0], loc[3][1], loc[3][2])

        simple_move_base(loc[instr.destination][0], loc[instr.destination][1], loc[instr.destination][2])
        rospy.set_param('/thesis/pepper_location', instr.destination)

        get_function(instr)

    return


if __name__ == '__main__':
    rospy.init_node('motion_planner', anonymous=True)

    # global const for action recognition
    pkg_dir = rospkg.RosPack().get_path('thesis')
    config_dir = pkg_dir + '/config/'
    human_info_dir = pkg_dir + '/human_info/'

    human_dict = load_human_info2dict(human_info_dir)
    action_cat = get_action_cat()

    rospy.set_param('/thesis/pepper_loc', 2)  # initial robot location to 'charge'

    step_t = 1.0  # time per time step (in second)

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

    func_symbol = {0: 'NOP', 1: 'chat', 2: 'encourage', 3: 'remind object', 4: 'remind schedule',
                   5: 'check human', 6: 'charge', 7: 'play videos', 8: 'play game', 9: 'emergency'}

    loc = [[office_x, office_y, office_yaw],
           [bedroom_x, bedroom_y, bedroom_yaw],
           [charge_x, charge_y, charge_yaw],
           [alley_x, alley_y, alley_yaw],
           [living_x, living_y, living_yaw],
           [dining_x, dining_y, dining_yaw],
           [greet_x, greet_y, greet_yaw],
           [emer_x, emer_y, emer_yaw]]

    goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                   'SUCCEEDED', 'ABORTED', 'REJECTED',
                   'PREEMPTING', 'RECALLING', 'RECALLED',
                   'LOST']
    # Start move_base
    if '/move_base' not in rosnode.get_node_names():
        rospy.loginfo('Start move_base.')
        subprocess.call('~/catkin_ws/src/pepper_try/scripts/start_move_base.sh', shell=True)
        time.sleep(1.5)

    rospy.wait_for_service('/move_base/make_plan', timeout=30)
    get_global_path = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/thesis/instruction_buffer', InstructionArray, motion_cb, queue_size=1)

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

    rospy.loginfo('Start Motion Planner !')
    rospy.spin()
