#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
This is for human identification
"""
import rospy
import rosgraph
import rospkg

from jsk_gui_msgs.msg import VoiceMessage
from tfpose_ros.msg import Persons
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from thesis.msg import Human
from rospy_message_converter import message_converter

import sys
import os
import qi

import cv2
import numpy as np
import pandas as pd
from numpy.linalg import norm
import yaml
import time
import pickle

# for demo
# Naoqi setting
if rospy.has_param("Pepper_ip"):
    pepper_ip = rospy.get_param("Pepper_ip")
else:
    print 'Pepper_ip is not given'
    pepper_ip = '192.168.0.184'
print 'Pepper_ip = ', pepper_ip

session = qi.Session()

try:
    session.connect("tcp://" + pepper_ip + ":" + str(9559))
except RuntimeError:
    print("tcp://" + pepper_ip + "\"on port" + str(9559) + ".")
    print("Please check your script arguments. Run with -h option for help.")
    sys.exit(1)

tts_service = session.service('ALTextToSpeech')
tts_service.setLanguage('English')
asr_service = session.service("ALAnimatedSpeech")
# End Naoqi setting
# end for demo


def get_ip(data):
    callerid = data._connection_header['callerid']  # type: 'str'
    master = rosgraph.Master('listener')
    ip = master.lookupNode(callerid)
    ip_num = ip.split(':')[1][2:]  # type: 'str'

    print 'ip_num = ', ip_num
    print 'data = ', data

    return ip_num


def get_joint_color(img, joints, j_features):
    """
    get the color of pixel where the joints are
    :param img: BGR input image for openpose, np.array(height, width, 3)
    :param joints: the joints from OpenPose in pixel unit
    :param j_features: lists of numbers of joints, [1, 2, 5]
    :return: 2D array of RGB colors, np.array([[b1, g1, r1], [b2, g2, r2], [b3, g3, r3]]), shape=(-1, 3)
    """
    if j_features is None:
        j_features = [1, 2, 5]

    colors = np.array([]).astype(np.int8)

    for j in j_features:
        if np.all(joints[j] > 0):
            colors = np.append(colors, img[joints[j, 1], joints[j, 0], :])

        else:
            colors = np.append(colors, np.array([-1, -1, -1]))

    colors = colors.reshape(-1, 3)

    return colors


def get_people_joints(human_pose):
    person_list = []  # a list of 2D array
    head_chest_range = 20  # distance threshold btw joint[0] and joint[1]
    part_number = 18

    for idx, person in enumerate(human_pose.persons):
        joints = np.ones((part_number, 2), dtype=np.int) * -1
        for i in range(len(human_pose.persons[idx].body_part)):
            part = human_pose.persons[idx].body_part[i]
            # Transform the joint points back to the position on the image
            joints[part.part_id, 0] = part.x * human_pose.image_w
            joints[part.part_id, 1] = part.y * human_pose.image_h

        # filtering person who is too far or no head
        if np.all(joints[0] > 0) and (np.all(joints[16] > 0) or np.all(joints[17] > 0)):  # person has nose and one ear
            if np.all(joints[1] > 0):  # person has chest
                if norm(joints[0] - joints[1]) > head_chest_range:  # filtering person who is too far
                    person_list.append(joints)

            elif np.all(joints[2] > 0):
                if norm(joints[0] - joints[2]) > head_chest_range:  # filtering person who is too far
                    person_list.append(joints)

            elif np.all(joints[5] > 0):
                if norm(joints[0] - joints[5]) > head_chest_range:  # filtering person who is too far
                    person_list.append(joints)

    return person_list


def color_dist(c1, c2):
    """
    Check the shirt color similarity based on cosine similarity.
    :param c1: int np.array with shape=(features, 3) in BGR order
    :param c2: int np.array with shape=(features, 3) in BGR order
    :return: a scalar of average color distance, ignoring the negative values
    """

    c_dis = np.zeros(c1.shape[0])  # color distance
    for i in range(len(c_dis)):
        if np.all(c1[i]) >= 0 and np.all(c2[i]) >= 0:
            r_mean = (c1[i][2] + c2[i][2]) / 2.
            d_r = c1[i][2] - c2[i][2]
            d_g = c1[i][1] - c2[i][1]
            d_b = c1[i][0] - c2[i][0]
            c_dis[i] = np.sqrt((512.+r_mean)*d_r*d_r/256. + 4.*d_g*d_g + (767.-r_mean)*d_b*d_b/256.)

    rospy.logdebug('color_dis: {0}'.format(np.mean(c_dis)))
    # if __debug__:
    #     print 'color distance = ', c_dis

    return np.mean(c_dis)


def identify_human(h_info, j_features, person_list):
    if h_info is None:
        h_info = load_human_info(rospkg.RosPack().get_path('thesis') + '/human_info/')

    if j_features is None:
        j_features = [1, 2, 5]

    if person_list is None:
        try:
            human_pose = rospy.wait_for_message('/thesis/human_pose', Persons, timeout=10)
        except rospy.exceptions.ROSException:
            rospy.logerr("Error when fetching human_pose.")
            return

        person_list = get_people_joints(human_pose)

    try:
        img_stitch = cv_bridge.imgmsg_to_cv2(rospy.wait_for_message('/thesis/img_stitching', Image, timeout=10), "bgr8")

    except rospy.exceptions.ROSException:
        rospy.logerr("Error when fetching img_stitching.")
        return

    for joints in person_list:
        identify_single_human(img_stitch, joints, h_info, j_features)

    return


def identify_single_human(img, joints, h_info, j_features, color_th=200.):
    """
    Check matching in the human dataset
    :param img: input images
    :param joints: human joints from openpose
    :param h_info: human in the dataset
    :param j_features: the joints for storing
    :param color_th: threshold of similarity
    :return: Human data structure or None
    """
    if h_info is None:
        h_info = load_human_info(rospkg.RosPack().get_path('thesis') + '/human_info/')

    if j_features is None:
        j_features = [1, 2, 5]

    colors = get_joint_color(img, joints, j_features)
    temp_sim = np.zeros(len(h_info))

    for i, human in enumerate(h_info):
        print 'color: ', colors
        print human.name, human.shirt_color
        temp_sim[i] = color_dist(colors, human.shirt_color)
        rospy.loginfo('{0} dis: {1}'.format(human.name, temp_sim[i]))

    human_result = h_info[np.argmin(temp_sim)] if np.min(temp_sim) < color_th else None

    if human_result is None:
        rospy.logwarn("New to this person.")

    else:
        rospy.loginfo('The person is {0}'.format(human_result.name))

    return human_result


def identify_voice(ip_dict, voice_msg):
    """
    Know who is talking.
    :param ip_dict: diction of Human() with ip asa index
    :param voice_msg: message form Tablet
    :return: Human()
    """
    if ip_dict is None:
        h_dict = load_human_info2dict(rospkg.RosPack().get_path('thesis') + '/human_info/')
        ip_dict = h_dict['ip']

    if voice_msg is None:
        voice_msg = rospy.wait_for_message('/Tablet/voice', VoiceMessage, timeout=10)

    voice_ip = get_ip(voice_msg)

    return ip_dict.get(voice_ip, None)


def get_human_from_name(name_dict, name):
    if name_dict is None:
        h_dict = load_human_info2dict(rospkg.RosPack().get_path('thesis') + '/human_info/')
        name_dict = h_dict['name']

    return name_dict.get(name, None)


def show_color(colors):
    """
    Show the cloth color
    :param colors: list of np.array([b, g, r])
    :return: showing a 50x50 image per color for t seconds
    """
    w = 50
    s = 5
    t = 1
    vis_color = np.ones((w, w*len(colors) + s * (len(colors)-1), 3)).astype(np.uint8) * 255

    for i, c in enumerate(colors):
        if np.any(c) == -1:  # invalid color
            i -= 1
            continue

        vis_color[:, i*(w+s):i*(w+s)+w, :] = np.tile(c, (w, w, 1))

    cv2.imshow('vis_color', vis_color)
    cv2.waitKey(1000*t)
    cv2.imwrite('/home/robot/pepper_data/result/id_color/P004.png', vis_color)

    return


def greeting_cb():
    rospy.set_param('/thesis/use_openpose', True)
    rospy.sleep(0.1)

    # Get human name
    asr_service.say('Hi, I am Pepper. Nice to meet you. What is your name?')
    print 'What is your name?'
    voice_msg = rospy.wait_for_message('/Tablet/voice', VoiceMessage)  # type: VoiceMessage
    print voice_msg

    ip_num = get_ip(voice_msg)
    name = voice_msg.texts[0].split(' ')[0]
    print 'ip = ', ip_num
    print 'name = ', name

    # Get the age of human
    age_flag = True
    temp_age = 0
    while age_flag:
        asr_service.say('How old are you?')
        voice_msg = rospy.wait_for_message('/Tablet/voice', VoiceMessage)  # type: VoiceMessage
        print 'Receive from phone: ', voice_msg
        _age = voice_msg.texts[0].split(' ')[0]  # type: VoiceMessage

        try:
            temp_age = int(_age)
            age_flag = False

        except ValueError:
            print 'Fail to recognize age from voice.'
            tts_service.say('Sorry, please try again.')
            time.sleep(1)
            continue

    # Get the gender of human
    while True:
        tts_service.say('Are you man or lady?')
        voice_msg = rospy.wait_for_message('/Tablet/voice', VoiceMessage)  # type: VoiceMessage
        print 'Receive from phone: ', voice_msg

        if voice_msg.texts[0].split(' ')[0] == 'man' or 'men':
            gen = 1
            break
        elif voice_msg.texts[0].split(' ')[0] == 'lady':
            gen = 0
            break
        else:
            tts_service.say('Sorry, please try again.')

    try:
        img_stitch = cv_bridge.imgmsg_to_cv2(rospy.wait_for_message('/thesis/img_stitching', Image, timeout=10), "bgr8")
        human_pose = rospy.wait_for_message('/thesis/human_pose', Persons, timeout=20)

    except rospy.exceptions.ROSException:
        return

    max_dis = 0
    max_joints = np.ones((part_num, 2), dtype=np.int) * -1
    for idx, person in enumerate(human_pose.persons):
        joints = np.ones((part_num, 2), dtype=np.int) * -1
        for i in range(len(human_pose.persons[idx].body_part)):
            part = human_pose.persons[idx].body_part[i]
            # Transform the joint points back to the position on the image
            joints[part.part_id, 0] = part.x * human_pose.image_w
            joints[part.part_id, 1] = part.y * human_pose.image_h

        # Pick person with longest distance of joint0 and joint1
        if np.all(joints[2] > 0) and np.all(joints[5] > 0):  # person has nose and one ear
            if norm(joints[2] - joints[5]) > max_dis:
                max_dis = norm(joints[2] - joints[5])
                max_joints = joints

    if np.all(max_joints == -1):
        print 'no human in the front.'

    else:
        # Show picked joint
        colors = get_joint_color(img_stitch, max_joints, joints_features)
        show_color(colors)

        # Create Human message and store in yaml format
        human = Human(name=name, ip=ip_num, gender=gen, age=temp_age,
                      shirt_color=colors, location=6, action=get_action_len())
        store_human_info(human)
        update_human_info_dict(human)

        respond = 'I got it, nice to meet you ' + name
        asr_service.say(respond)
    rospy.set_param('/thesis/use_openpose', False)
    return


def greeting_eval():
    try:
        img_stitch = cv_bridge.imgmsg_to_cv2(rospy.wait_for_message('/thesis/img_stitching', Image, timeout=10), "bgr8")
        human_pose = rospy.wait_for_message('/thesis/human_pose', Persons, timeout=10)

    except rospy.exceptions.ROSException:
        return

    max_dis = 0
    max_joints = np.ones((part_num, 2), dtype=np.int) * -1
    for idx, person in enumerate(human_pose.persons):
        joints = np.ones((part_num, 2), dtype=np.int) * -1
        for i in range(len(human_pose.persons[idx].body_part)):
            part = human_pose.persons[idx].body_part[i]
            # Transform the joint points back to the position on the image
            joints[part.part_id, 0] = part.x * human_pose.image_w
            joints[part.part_id, 1] = part.y * human_pose.image_h

        # Pick person with longest distance of joint0 and joint1
        if np.all(joints[2] > 0) and np.all(joints[5] > 0):  # person has nose and one ear
            if norm(joints[2] - joints[5]) > max_dis:
                max_dis = norm(joints[2] - joints[5])
                max_joints = joints

    if np.all(max_joints == -1):
        print 'no human in the front.'

    else:
        # Show picked joint
        colors = get_joint_color(img_stitch, max_joints, joints_features)
        show_color(colors)

        # Create Human message and store in yaml format
        name = 'Bob'
        human = Human(name=name, ip='192.168.50.888', gender=1, age=24,
                      shirt_color=colors, location=7, action=get_action_len())
        store_human_info(human)
        update_human_info_dict(human)

        respond = 'I got it, nice to meet you ' + name
        rospy.loginfo(respond)


def store_human_info(in_human):
    human_info_dir = rospkg.RosPack().get_path('thesis') + '/human_info/'
    temp_human = Human(name=in_human.name,
                       ip=in_human.ip,
                       shirt_color=in_human.shirt_color.tolist(),
                       location=in_human.location,
                       action=in_human.action)

    human_dic = message_converter.convert_ros_message_to_dictionary(temp_human)  # transform into dictionary type

    f_name = human_info_dir + in_human.name + '.yaml'  # type: str
    with open(f_name, 'w') as f:
        yaml.dump(human_dic, f)

    return


def update_human_info_dict(in_human):
    # Update human_info list and human_dict

    human_info_dir = rospkg.RosPack().get_path('thesis') + '/human_info/'
    if 'human_info.pkl' in os.listdir(human_info_dir):
        h_list = load_human_info(human_info_dir)
        h_list.append(in_human)

        with open('human_info.pkl', 'wb') as fin:
            pickle.dump(h_list, fin)

        name_dict = dict()
        ip_dict = dict()
        h_dict = dict()
        for h in h_list:
            name_dict[h.name] = h
            ip_dict[h.ip] = h

        h_dict['name'] = name_dict
        h_dict['ip'] = ip_dict

        with open(human_info_dir+'human_dict.pkl', 'wb') as fin:
            pickle.dump(h_dict, fin)

    return


def load_human_info(human_info_dir):
    """
    load human info for human identification.
    :return: A list of stored human beings.
    """

    if 'human_info.pkl' in os.listdir(human_info_dir):
        with open(human_info_dir+'human_info.pkl', 'rb') as fout:
            h_list = pickle.load(fout)

    else:
        yaml_list = os.listdir(human_info_dir)
        print 'Current human in dataset: ', yaml_list

        h_list = []

        if yaml_list is None:
            print 'No human in robot memory.'
        else:
            for f in yaml_list:
                if f.endswith('.yaml'):
                    temp = yaml.load(open(human_info_dir + f))
                    temp_shirt_color = np.array(temp['shirt_color'])
                    human_msg = message_converter.convert_dictionary_to_ros_message('thesis/Human', temp)
                    human_msg.shirt_color = temp_shirt_color

                    h_list.append(human_msg)

            with open(human_info_dir+'human_info.pkl', 'wb') as fin:
                pickle.dump(h_list, fin)

    return h_list


def load_human_info2dict(human_info_dir):
    if 'human_dict.pkl' in os.listdir(human_info_dir):
        with open(human_info_dir+'human_dict.pkl', 'rb') as fout:
            human_dict = pickle.load(fout)
    else:
        h_list = load_human_info(human_info_dir)
        name_dict = dict()
        ip_dict = dict()
        human_dict = dict()
        for h in h_list:
            name_dict[h.name] = h
            ip_dict[h.ip] = h.name

        human_dict['name'] = name_dict
        human_dict['ip'] = ip_dict

    return human_dict


def get_action_len():
    config_dir = rospkg.RosPack().get_path('thesis') + '/config/'
    hand_acts = pd.read_csv(config_dir + 'hand_actions.csv', sep=',')  # DataFrame
    action_cat = hand_acts.columns.to_list()  # category of actions
    return len(action_cat)


if __name__ == '__main__':
    rospy.init_node('human_id', log_level=rospy.INFO)
    part_num = 18
    joints_features = [1, 2, 5]
    cv_bridge = CvBridge()

    is_eval = False
    if is_eval:
        rospy.loginfo('Evaluation ...')
        greeting_eval()

    else:
        # Naoqi setting
        if rospy.has_param("Pepper_ip"):
            pepper_ip = rospy.get_param("Pepper_ip")
        else:
            print 'Pepper_ip is not given'
            pepper_ip = '192.168.0.184'
        print 'Pepper_ip = ', pepper_ip

        session = qi.Session()

        try:
            session.connect("tcp://" + pepper_ip + ":" + str(9559))
        except RuntimeError:
            print("tcp://" + pepper_ip + "\"on port" + str(9559) + ".")
            print("Please check your script arguments. Run with -h option for help.")
            sys.exit(1)

        tts_service = session.service('ALTextToSpeech')
        tts_service.setLanguage('English')
        asr_service = session.service("ALAnimatedSpeech")
        # End Naoqi setting

        rospy.loginfo('human_id start!')

        greet_msg = rospy.wait_for_message('/Tablet/voice', VoiceMessage)  # type: VoiceMessage
        print greet_msg
        if greet_msg.texts[0] == 'hello' or 'halo':
            greeting_cb()
