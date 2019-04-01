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
    name = voice_msg.texts[0].split(' ')

    print 'ip = ', ip_num
    print 'name = ', name[0]

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
        # cv2.circle(img_stitch, (max_joints[1, 0], max_joints[1, 1]), 5, (255, 0, 0), 2)
        # cv2.circle(img_stitch, (max_joints[2, 0], max_joints[2, 1]), 5, (0, 255, 0), 2)
        # cv2.circle(img_stitch, (max_joints[5, 0], max_joints[5, 1]), 5, (0, 0, 255), 2)
        # cv2.imshow('max_joint', img_stitch)
        # cv2.waitKey(5000)

        color_1 = img_stitch[max_joints[1, 1], max_joints[1, 0], :] if np.all(max_joints[1] > 0) else -1
        color_2 = img_stitch[max_joints[2, 1], max_joints[2, 0], :] if np.all(max_joints[2] > 0) else -1
        color_5 = img_stitch[max_joints[5, 1], max_joints[5, 0], :] if np.all(max_joints[5] > 0) else -1

        show_color([color_2, color_1, color_5])

        respond = 'I got it, nice to meet you ' + name[0]
        as_service.say(respond)

    return


if __name__ == '__main__':
    rospy.init_node('human_id', log_level=rospy.INFO)
    part_num = 18
    cv_bridge = CvBridge()

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

    tts_service = session.service('ALTextToSpeech')
    tts_service.setLanguage('English')
    as_service = session.service("ALAnimatedSpeech")

    # End Naoqi setting

    rospy.loginfo('human_id start!')

    greet_msg = rospy.wait_for_message('/Tablet/voice', VoiceMessage)  # type: VoiceMessage
    print greet_msg
    if greet_msg.texts[0] == 'hello' or 'halo':
        greeting_cb()
