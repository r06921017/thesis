#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This is for human identification
"""
import rospy
import rosgraph
from jsk_gui_msgs.msg import VoiceMessage
from tfpose_ros.msg import Persons
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from thesis.msg import Human
from rospy_message_converter import message_converter

import sys
import qi

import cv2
import numpy as np
from numpy.linalg import norm
import yaml


def get_ip(data):
    callerid = data._connection_header['callerid']  # type: 'str'
    master = rosgraph.Master('listener')
    ip = master.lookupNode(callerid)
    ip_num = ip.split(':')[1][2:]  # type: 'str'

    print 'ip_num = ', ip_num
    print 'data = ', data

    return ip_num


def get_joint_color(img, joints, joints_list):
    """
    get the color of pixel where the joints are
    :param img: BGR input image for openpose, np.array(height, width, 3)
    :param joints: the joints from OpenPose in pixel unit
    :param joints_list: lists of numbers of joints, [1, 2, 5]
    :return: 2D array of RGB colors, np.array([[b1, g1, r1], [b2, g2, r2], [b3, g3, r3]]), shape=(-1, 3)
    """
    colors = np.array([]).astype(np.int8)
    print 'joint_list = ', joints_list

    for j in joints_list:
        if np.all(joints[j] > 0):
            colors = np.append(colors, img[joints[j, 1], joints[j, 0], :])

        else:
            colors = np.append(colors, np.array([-1, -1, -1]))

    colors = colors.reshape(-1, 3)

    return colors


def show_color(colors):
    """
    Show the cloth color
    :param colors: list of np.array([b, g, r])
    :return: showing a 50x50 image per color for t seconds
    """
    w = 50
    s = 5
    t = 2
    vis_color = np.ones((w, w*len(colors) + s * (len(colors)-1), 3)).astype(np.uint8) * 255

    for i, c in enumerate(colors):
        if np.any(c) == -1:  # invalid color
            i -= 1
            continue

        vis_color[:, i*(w+s):i*(w+s)+w, :] = np.tile(c, (w, w, 1))

    print vis_color.shape
    cv2.imshow('vis_color', vis_color)
    cv2.waitKey(1000*t)

    return


def greeting_cb():
    tts_service.say('Hi, I am Pepper. Nice to meet you. What is your name?')
    print 'What is your name?'
    voice_msg = rospy.wait_for_message('/Tablet/voice', VoiceMessage)  # type: VoiceMessage
    print voice_msg

    ip_num = get_ip(voice_msg)
    name = voice_msg.texts[0].split(' ')[0]

    print 'ip = ', ip_num
    print 'name = ', name

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
        human = Human(name=name, ip=ip_num, shirt_color=colors.tolist(), location='greet')
        human_dic = message_converter.convert_ros_message_to_dictionary(human)  # transform into dictionary type
        f_name = '/home/robot/catkin_ws/src/thesis/human_info/' + name + '.yaml'
        with open(f_name, 'w') as f:
            yaml.dump(human_dic, f)

        respond = 'I got it, nice to meet you ' + name
        as_service.say(respond)

    return


if __name__ == '__main__':
    rospy.init_node('human_id', log_level=rospy.INFO)
    part_num = 18
    joints_features = [1, 2, 5]
    cv_bridge = CvBridge()

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
    as_service = session.service("ALAnimatedSpeech")
    # End Naoqi setting

    rospy.loginfo('human_id start!')

    greet_msg = rospy.wait_for_message('/Tablet/voice', VoiceMessage)  # type: VoiceMessage
    print greet_msg
    if greet_msg.texts[0] == 'hello' or 'halo':
        greeting_cb()
