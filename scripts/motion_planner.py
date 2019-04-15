"""
Make robot move from node to node.
"""

import subprocess
from math import atan2

import actionlib
import dynamic_reconfigure.client
import rosnode
from actionlib_msgs.msg import *
from genpy import Duration, Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from thesis.msg import *
from typing import List, Union

from human_id import *


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
            sac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

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

    if norm(rx-des_x, ry-des_y) < 0.5 and abs(float(ryaw - des_yaw)) < 0.3:
        print 'is in destination'
        return True
    else:
        print 'not in destination'
        return False


def get_function(instr):

    if instr.function == 0:  # NOP
        say_str = 'Hello ' + instr.target + ', what can I do for you?'
        tts_service.say(say_str)

    elif instr.function == 1:  # chat
        say_str = 'Greetings, ' + instr.target + 'how do you do?'
        tts_service.say(say_str)
        voice_msg = rospy.wait_for_message('/Tablet/voice', VoiceMessage)  # type: VoiceMessage
        human = identify_voice(human_info, voice_msg)

        if human is None:
            greeting_cb()
        else:
            # human say: Thank you for chatting with me
            tts_service.say('Well, I am glad to help.')

    elif instr.function == 2:  # remind object
        remind_obj()

    elif instr.function == 8:  # emergency
        tts_service.say('I will call Li-Pu for help.')
        simple_move_base(emer_x, emer_y, emer_yaw)
        say_str = 'Li-Pu, ' + instr.target + 'may need your help. Please be hurry!'
        asr_service.say(say_str)
        simple_move_base(loc[instr.destination][0], loc[instr.destination][1], loc[instr.destination][2])

    return


def remind_obj():

    return


def motion_cb(data):
    """
    Navigation based on instructions
    :param data: InstructionArray
    :return: None
    """
    # move to destination from node to node
    for instr in data:
        simple_move_base(loc[instr.destination][0], loc[instr.destination][1], loc[instr.destination][2])

    return


if __name__ == '__main__':
    rospy.init_node('motion_planner', anonymous=True)

    # global const for action recognition
    pkg_dir = rospkg.RosPack().get_path('thesis')
    config_dir = pkg_dir + '/config/'
    human_info_dir = rospkg.RosPack().get_path('thesis') + '/human_info/'
    human_info = load_human_info(human_info_dir)  # type: List[Human]

    office_x, office_y, office_yaw = 0.716, -0.824,  1.927  # location: 0
    bedroom_x, bedroom_y, bedroom_yaw = 4.971, -0.005,  2.026  # 1
    charge_x, charge_y, charge_yaw = 5.024, -0.318, -2.935
    alley_x, alley_y, alley_yaw = 5.140, -3.713,  2.017
    living_x, living_y, living_yaw = 3.397, -5.461,  1.927
    dining_x, dining_y, dining_yaw = 6.258, -3.560,  1.353
    emer_x, emer_y, emer_yaw = 5.294, -3.869, -1.165  # emergency

    loc = [[office_x, office_y, office_yaw],
          [bedroom_x, bedroom_y, bedroom_yaw],
          [charge_x, charge_y, charge_yaw],
          [alley_x, alley_y, alley_yaw],
          [living_x, living_y, living_yaw],
          [dining_x, dining_y, dining_yaw],
          [emer_x, emer_y, emer_yaw]]

    goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                   'SUCCEEDED', 'ABORTED', 'REJECTED',
                   'PREEMPTING', 'RECALLING', 'RECALLED',
                   'LOST']

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
    # End Naoqi setting

    rospy.spin()
