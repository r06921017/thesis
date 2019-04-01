#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This is for human identification
"""
import rospy
import rospkg
import rosgraph
from genpy import Message
from jsk_gui_msgs.msg import VoiceMessage
from tfpose_ros.msg import Persons
from darknet_ros_msgs.msg import *

import qi
import numpy as np
import pandas as pd
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


def greeting_cb(data):
    # tts_service.say('Hi, I am Pepper. Nice to meet you. What is your name?')
    print 'What is your name?'
    try:
        voice_msg = rospy.wait_for_message('/Tablet/voice', VoiceMessage, timeout=40)  # type: VoiceMessage
    except rospy.exceptions.ROSException:
        # tts_service.say('Sorry, can you speak again?')
        return

    ip_num = get_ip(voice_msg)
    name = voice_msg.texts[0].split(' ')

    print ip_num
    print name

    return


def people_detected_cb(value):  # for qi session
    print 'people detected callback'
    if not value:
        print 'No human'

    else:
        print 'value = ', value
        print 'TimeStamp_second = ', value[0]
        print 'PersonData = ', value[1]

    return


if __name__ == '__main__':
    rospy.init_node('human_id', log_level=rospy.INFO)
    rospy.loginfo('human_id start!')

    rospy.Subscriber('/Tablet/voice', VoiceMessage, greeting_cb, queue_size=1)

    # Naoqi setting
    global pepper_ip
    if rospy.has_param("Pepper_ip"):
        pepper_ip = rospy.get_param("Pepper_ip")
    else:
        print 'Pepper_ip is not given'
        pepper_ip = '192.168.0.183'
    print 'Pepper_ip = ', pepper_ip

    global motion_service, posture_service, tts_service

    session = qi.Session()

    try:
        session.connect("tcp://" + pepper_ip + ":" + str(9559))
    except RuntimeError:
        print("tcp://" + pepper_ip + "\"on port" + str(9559) + ".")
        print("Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    motion_service = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    tts_service = session.service("ALTextToSpeech")
    tts_service.setLanguage("English")
    as_service = session.service("ALAnimatedSpeech")
    # vea_service = session.service("ALVoiceEmotionAnalysis")

    # pp_service = session.service("ALPeoplePerception")  # no use
    mem_service = session.service("ALMemory")
    qi_sub = mem_service.subscriber("PeoplePerception/PeopleDetected")
    qi_sub.signal.connect(people_detected_cb)
    # End Naoqi setting

    # rate = rospy.Rate(10)  # Hz
    # while not rospy.is_shutdown():
    #     rate.sleep()
    rospy.spin()
