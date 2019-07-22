#!/usr/bin/env python
# -*- coding: utf8 -*-

import subprocess
from math import atan2, pi
import actionlib
import dynamic_reconfigure.client
import rosnode
from actionlib import SimpleActionClient
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from perception.human_id import *
from perception.action_recognition import *


# Start move_base
rospy.loginfo('Start move_base.')
subprocess.call('~/catkin_ws/src/thesis/scripts/motion/start_move_base.sh', shell=True)
rospy.sleep(1.5)
rospy.loginfo('move_base launch!')

rospy.wait_for_service('/move_base/make_plan', timeout=5)
get_global_path = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
               'SUCCEEDED', 'ABORTED', 'REJECTED',
               'PREEMPTING', 'RECALLING', 'RECALLED',
               'LOST']

office_x, office_y, office_yaw = 2.045, -0.15, 2.69  # yaw2.45  # 0.716, -0.824, 1.927  # location: 0
bedroom_x, bedroom_y, bedroom_yaw = 4.692, 0.329, 1.964  # 4.681, 0.271, 1.941  # 1
charge_x, charge_y, charge_yaw = 4.487, -0.473, -2.693  # 5.024, -0.318, -2.935  # 2
alley1_x, alley1_y, alley1_yaw = 3.827, -1.191, -1.164  # 3
alley2_x, alley2_y, alley2_yaw = 5.140, -3.713, 2.017  # 4
living_x, living_y, living_yaw = 3.201, -5.051, 1.46  # 3.397, -5.461, 1.927  # 5
dining_x, dining_y, dining_yaw = 6.172, -3.271, 1.496  # 6
greet_x, greet_y, greet_yaw = 3.201, -5.051, -0.491  # 7, greeting for welcome
# emer_x, emer_y, emer_yaw = 5.065, -3.795, -1.195  # 8, emergency
emer_x, emer_y, emer_yaw = 5.058, -4.036, -1.195  # 8, emergency


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

# for function operation
human_info_dir = rospkg.RosPack().get_path('thesis') + '/'
human_dict = load_human_info2dict(human_info_dir)
action_cat = get_action_cat()
cv_bridge = CvBridge()
# end function operation

# Naoqi setting
if rospy.has_param("Pepper_ip"):
    pepper_ip = rospy.get_param("Pepper_ip")
else:
    print 'Pepper_ip is not given'
    pepper_ip = '192.168.50.99'
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


def get_function(instr, in_sac=None):
    rospy.set_param('/thesis/face_track', True)  # start face tracking
    function_start_time = time.time()

    if instr.function == 0:  # NOP
        say_str = 'Hello ' + instr.target + ', what can I do for you?'
        asr_service.say(say_str)

    elif instr.function == 1:  # chat
        if instr.status == 2 and instr.type == 0:  # negative emotion launched by human
            tts_service.say('What happened?')  # encourage for break up
            voice_msg = rospy.wait_for_message('/Tablet/voice', VoiceMessage)  # type: VoiceMessage
            if 'break' and 'up' in voice_msg.texts[0].split(' '):
                asr_service.say('Cheer up! you deserve a better one.')
                time.sleep(0.5)
                asr_service.say('I will be with you whenever you are down.')
                voice_msg = rospy.wait_for_message('/Tablet/voice', VoiceMessage)
                if 'Thank' or 'thank' or 'Thanks' in voice_msg.texts[0].split(' '):
                    asr_service.say('No problem, my friend.')
                    posture_service('StandInit', 0.6)

        say_str = 'Greetings, ' + instr.target + 'how do you do?'
        asr_service.say(say_str)
        voice_msg = rospy.wait_for_message('/Tablet/voice', VoiceMessage)  # type: VoiceMessage
        human = identify_voice(human_dict['ip'], voice_msg)

        if human is None:  # add new person into human_info
            greeting_cb()

        else:
            # human say: Thank you for chatting with me
            asr_service.say('Well, I am glad to help.')

    elif instr.function == 2:  # remind object
        obj_loc = remind_obj(instr.target)  # list of locations of given objects
        if obj_loc is None:  # no locations founds
            temp_str = 'I forget where ' + instr.target + 'is. I will find it later.'
            asr_service.say(temp_str)
            # TODO: add find object function

    elif instr.function == 3:  # remind schedule
        temp_str = 'Hello '+instr.target+', your friend is coming at three thirty.'
        tts_service.say(temp_str)
        time.sleep(3)
        asr_service.say('No problem. Have fun then.')
        # target_human = get_human_from_name(name_dict=human_dict['name'], name=instr.target)
        # cur_weekday = datetime.datetime.today().weekday()
        # cur_hour = datetime.datetime.now().hour
        # # cur_min = datetime.datetime.now().minute
        # sch_list = []
        # for sch in target_human.schedules:
        #     if sch.start_day < cur_weekday < sch.end_day and cur_hour < sch.start_hour:
        #         sch_str = sch.activity + 'at ' + sch.start_hour + ', '
        #         sch_list.append(sch_str)
        #
        # # formulate as sentence for robot
        # temp_str = 'You need to '
        # for i, sch_str in enumerate(sch_list):
        #     if i == len(sch_list) - 1:
        #         temp_str += 'and '
        #     temp_str += sch_str

    elif instr.function == 4:  # check human
        rospy.set_param('/thesis/use_openpose', True)  # start and save action
        rospy.set_param('/thesis/action_on', True)  # start and save action
        time.sleep(3)
        rospy.set_param('/thesis/action_on', False)  # stop
        rospy.set_param('/thesis/use_openpose', False)

    elif instr.function == 5:  # charge
        tts_service.say('I need to charge, can anyone help me, please?')
        time.sleep(2)  # wait to charge 10 seconds

    elif instr.function == 6:  # play videos
        asr_service.say('Would you like some music?')
        tablet_service.playVideo("https://www.youtube.com/watch?v=lmNHeu7DB28")
        time.sleep(instr.duration - (time.time() - function_start_time) - 0.5)

    elif instr.function == 7:  # play games
        asr_service.say('Let\'s play a game!')
        time.sleep(instr.duration - (time.time() - function_start_time) - 0.5)

    elif instr.function == 8:  # emergency
        tts_service.say(instr.source+', are you Okay? What is going on?')
        rospy.sleep(3.5)
        asr_service.say('I will call Alfred for help. Please wait for a second.')
        rospy.sleep(0.25)
        if in_sac is None:
            _temp_sac = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # type: SimpleActionClient
            simple_move_base(_temp_sac, emer_x, emer_y, emer_yaw)
        else:
            simple_move_base(in_sac, emer_x, emer_y, emer_yaw)

        say_str = 'Alfred, ' + instr.source + 'may need your help. Please come to the bedroom as soon as possible.'
        asr_service.say(say_str)
        rospy.sleep(1)
        # simple_move_base(in_sac, loc[instr.destination][0], loc[instr.destination][1], loc[instr.destination][2])

    elif instr.function == 9:  # report to source
        # target_human = get_human_from_name(name_dict=human_dict['name'], name=instr.target)
        # temp_str = target_human.name + ' is ' + action_cat[target_human.action]
        temp_str = 'By the way, I think Alex is working'
        asr_service.say(temp_str)
        rospy.sleep(2)
        rospy.sleep(0.1)

    elif instr.function == 10:  # welcome guests
        greeting_cb()

    rospy.set_param('/thesis/face_track', False)

    posture_service.goToPosture("StandInit", 0.8)

    # sleep until duration ends
    if instr.function != 8:
        if instr.duration - (time.time() - function_start_time) > 0.:
            time.sleep(instr.duration - (time.time() - function_start_time))
    else:
        time.sleep(2)

    return


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


def relaunch_move_base():
    # noinspection PyBroadException
    try:
        subprocess.call('~/catkin_ws/src/thesis/scripts/motion/stop_move_base.sh', shell=True)
        rospy.sleep(0.5)
    except Exception:
        pass
    subprocess.call('~/catkin_ws/src/thesis/scripts/motion/start_move_base.sh', shell=True)
    rospy.sleep(0.5)
    return


def simple_rotate(delta_rad):
    if delta_rad < -pi:
        _angle = delta_rad + 2*pi
    elif delta_rad > pi:
        _angle = delta_rad - 2*pi
    else:
        _angle = delta_rad
    motion_service.moveTo(0.0, 0.0, _angle, 1)
    return


def simple_move_base(sac, dest_x, dest_y, dest_yaw, inflation_radius=0.7, with_rotation_first=True, path_th=4):
    rospy.logdebug('inflation_radius = {0}'.format(inflation_radius))
    rospy.loginfo('destination = {0} {1} {2}'.format(dest_x, dest_y, dest_yaw))

    # robot is static
    cur_x, cur_y, cur_yaw = get_cur_pos(pos_topic='amcl_pose', t=0.5)
    rospy.logdebug('cur_x, cur_y, cur_yaw = {0} {1} {2}'.format(cur_x, cur_y, cur_yaw))
    tan_yaw = atan2(dest_y-cur_y, dest_x-cur_x)

    if not isin_dest(cur_x, cur_y, cur_yaw, dest_x, dest_y, tan_yaw):

        start = PoseStamped()
        start.header.frame_id = 'map'
        start.pose.position.x = cur_x
        start.pose.position.y = cur_y

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = dest_x
        goal.pose.position.y = dest_y

        if '/move_base' in rosnode.get_node_names():
            rospy.loginfo('move_base node succeed!')

            change_local_costmap_radius(inflation_radius)
            rospy.sleep(0.1)

            if with_rotation_first:
                # noinspection PyBroadException
                try:
                    temp_global_path = get_global_path(start, goal, 0.1)
                except Exception:
                    temp_global_path = []

                if len(temp_global_path.plan.poses) > path_th:
                    next_location = [temp_global_path.plan.poses[path_th].pose.position.x,
                                     temp_global_path.plan.poses[path_th].pose.position.y]

                    # Update tan_yaw for smooth navigation
                    tan_yaw = atan2(dest_y - temp_global_path.plan.poses[-path_th].pose.position.y,
                                    dest_x - temp_global_path.plan.poses[-path_th].pose.position.x)

                elif len(temp_global_path.plan.poses) > 0:
                    next_location = [temp_global_path.plan.poses[-1].pose.position.x,
                                     temp_global_path.plan.poses[-1].pose.position.y]

                    # Update tan_yaw for smooth navigation
                    tan_yaw = atan2(dest_y - temp_global_path.plan.poses[-1].pose.position.y,
                                    dest_x - temp_global_path.plan.poses[-1].pose.position.x)
                else:
                    next_location = [dest_x, dest_y]

                rospy.logwarn('next_location: {0}'.format(next_location))

                theta = atan2(next_location[1] - cur_y, next_location[0] - cur_x)  # the angle to path direction
                rospy.logwarn('rotation first theta: {0}'.format(theta))

                simple_rotate(theta-cur_yaw)
                posture_service.goToPosture("StandInit", 0.5)
                rospy.loginfo('rotation finish')

            rospy.loginfo('tan_yaw = {0}'.format(tan_yaw))

            # simple move if the goal is less than 1.0 (m)
            if norm([dest_x - cur_x, dest_y - cur_y]) < 1.0:
                if with_rotation_first:
                    motion_service.moveTo(float(norm([dest_x - cur_x, dest_y - cur_y])), 0, 0, 2)

                else:
                    motion_service.moveTo(dest_x - cur_x, dest_y - cur_y, tan_yaw - cur_yaw, 2)
                return

            else:
                subprocess.call('rosservice call /move_base/clear_costmaps', shell=True)  # clear local costmap
                rospy.sleep(0.1)
                within_time = run_movebase(sac, dest_x, dest_y, tan_yaw)
                rospy.loginfo('within_time: {0}'.format(within_time))

                if not within_time:
                    rospy.logwarn('Timed out achieving goal.')
                    # tts_service.say('Timed out achieving goal.')

                    # noinspection PyBroadException
                    try:
                        # error handling
                        _scan_range = rospy.wait_for_message('/scan', LaserScan, timeout=0.5).ranges
                        rospy.logwarn('current laser ranges: {0}'.format(_scan_range))
                        obs_dir = cal_laser_range(_scan_range)
                        move_recover(obs_dir)
                    except Exception:
                        pass

                    shutdown()
                    relaunch_move_base()
                    rospy.sleep(0.25)
                    simple_move_base(sac=sac, dest_x=dest_x, dest_y=dest_y, dest_yaw=dest_yaw,
                                     inflation_radius=inflation_radius, with_rotation_first=False, path_th=4)

                else:
                    state = sac.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        rospy.loginfo('Goal succeed!')
                        rospy.set_param('/thesis/reach', True)

                    elif state == GoalStatus.ABORTED:
                        rospy.logwarn('Goal failed with error code:' + str(goal_states[state]))

                        # noinspection PyBroadException
                        try:
                            # error handling
                            _scan_range = rospy.wait_for_message('/scan', LaserScan, timeout=0.5).ranges
                            rospy.logwarn('current laser ranges: {0}'.format(_scan_range))
                            obs_dir = cal_laser_range(_scan_range)
                            move_recover(obs_dir)
                        except Exception:
                            pass

                        shutdown()
                        relaunch_move_base()
                        rospy.sleep(0.25)
                        inflation_radius -= 0.1
                        simple_move_base(sac=sac, dest_x=dest_x, dest_y=dest_y, dest_yaw=dest_yaw,
                                         inflation_radius=inflation_radius, with_rotation_first=False, path_th=4)
                    else:
                        shutdown()
                        # relaunch_move_base()

        # else:
        #     relaunch_move_base()
        #     rospy.sleep(0.5)

    else:
        rospy.set_param('/thesis/reach', True)

    return


def cal_laser_range(in_range):
    front_scan = list()
    left_scan = list()
    right_scan = list()

    obs_dict = {0: 'right', 1: 'front', 2: 'left'}

    for i in range(10):
        if 0.1 < in_range[i] < 1.5:
            right_scan.append(in_range[i])
        if 0.1 < in_range[i+25] < 1.5:
            front_scan.append(in_range[i+25])
        if 0.1 < in_range[i+50] < 1.5:
            left_scan.append(in_range[i+50])

    if len(right_scan) > 0:
        right_val = np.mean(right_scan)
    else:
        right_val = 10

    if len(front_scan) > 0:
        front_val = np.mean(front_scan)
    else:
        front_val = 10

    if len(left_scan) > 0:
        left_val = np.mean(left_scan)
    else:
        left_val = 10

    obs_dir = np.nanargmin(np.array([right_val, front_val, left_val]))
    rospy.logwarn('Obstacle exists in {0}'.format(obs_dict[obs_dir]))

    return obs_dir  # 0: right, 1: front, 2: left


def move_recover(in_obs_dir, theta=np.pi/4.0):
    rospy.logwarn('move_recover')
    motion_service.moveTo(-0.1, 0, 0, 1)
    if in_obs_dir == 0:  # right
        # tts_service.say('Turn left')
        simple_rotate(theta)
    elif in_obs_dir == 1:  # front
        # tts_service.say('Move back')
        motion_service.moveTo(-0.25, 0, 0, 1)
    elif in_obs_dir == 2:  # left
        # tts_service.say('Turn right')
        simple_rotate(-theta)
    rospy.sleep(0.1)
    return


def shutdown():
    rospy.loginfo('Shutdown the robot ...')
    for _ in range(4):
        cmd_vel_pub.publish(Twist())
    # subprocess.call('rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}', shell=True)
    cancel_pub.publish(GoalID())
    rospy.sleep(0.01)

    # rospy.sleep(1)
    # rospy.loginfo('canceling the goal ...')
    # sac.cancel_goal()
    return


def run_movebase(sac, x, y, yaw):
    rospy.loginfo('run_movebase start!')
    # Transform euler angle to quaternion
    _quat = quaternion_from_euler(0.0, 0.0, yaw)

    # create goal
    goal = MoveBaseGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.x = _quat[0]
    goal.target_pose.pose.orientation.y = _quat[1]
    goal.target_pose.pose.orientation.z = _quat[2]
    goal.target_pose.pose.orientation.w = _quat[3]
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    # noinspection PyBroadException
    try:
        sac.wait_for_server()
        sac.send_goal(goal)
        within_time = sac.wait_for_result(rospy.Duration(50))  # 50 sec
        rospy.sleep(0.5)
    except Exception:
        rospy.logerr('wait_for_server failed ...')
        within_time = False

    return within_time


def isin_dest(rx, ry, ryaw, des_x, des_y, des_yaw):
    dist_th = 0.6  # 0.3
    angle_th = 0.5  # 0.15
    if norm([rx - des_x, ry - des_y]) < dist_th and abs(float(ryaw - des_yaw)) < angle_th:
        rospy.loginfo('is in destination')
        return True
    else:
        rospy.loginfo('not in destination')
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

    for i in range(5):
        initial_pose_pub.publish(initial_pose)
        rospy.sleep(0.1)

    rospy.set_param('/thesis/pepper_location', init_location)

    rospy.loginfo('initial pose sent!')
    return


if __name__ == '__main__':
    rospy.init_node('robot_motions', log_level=rospy.INFO)
    rospy.loginfo('robot_motions start!')
    rospy.sleep(0.2)

    temp_symbol = 2
    dest_symbol = 6
    set_initial_pose(loc[temp_symbol][0], loc[temp_symbol][1], loc[temp_symbol][2], temp_symbol)
    rospy.sleep(1)
    temp_sac = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # type: SimpleActionClient

    start_time = time.time()
    simple_move_base(temp_sac, loc[dest_symbol][0], loc[dest_symbol][1], loc[dest_symbol][2])
    motion_service.moveTo(0.0, 0.0, loc[dest_symbol][2]-get_cur_pos('/amcl_pose', 0.5)[2])
    rospy.loginfo('Moving Time: {0}'.format(time.time() - start_time))
