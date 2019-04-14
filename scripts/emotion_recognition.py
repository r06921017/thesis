#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import sys
import qi
import time


def emotion_recognition():
    """
    :return: positive, neutral, negative, unknown
    """
    tts_service.say("Fuck you, Charlie.")
    time.sleep(3)
    print 'emotion = ', mood_service.getEmotionalReaction()
    return


if __name__ == '__main__':
    # Naoqi setting
    # if rospy.has_param("Pepper_ip"):
    #     pepper_ip = rospy.get_param("Pepper_ip")
    # else:
    #     print 'Pepper_ip is not given'
    #     pepper_ip = '192.168.0.184'
    # print 'Pepper_ip = ', pepper_ip

    pepper_ip = '192.168.0.184'
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
    mood_service = session.service("ALMood")
    # End Naoqi setting

    emotion_recognition()
