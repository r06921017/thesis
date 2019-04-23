#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
This is for human action recognition and human identification
"""
import rospy
import rospkg
import numpy as np
import pandas as pd
from numpy.linalg import norm
from tfpose_ros.msg import Persons
from darknet_ros_msgs.msg import *
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from human_id import load_human_info, get_people_joints, identify_single_human, store_human_info

np.set_printoptions(precision=2)


def isin_bbox(pt, xmin, xmax, ymin, ymax):
    """
    :param pt: np.array([x, y])
    :param xmin: int
    :param xmax: int
    :param ymin: int
    :param ymax: int
    :return: bool
    """
    if (xmin <= pt[0] <= xmax) and (ymin <= pt[1] <= ymax):
        # print 'human in box'
        return True
    else:
        return False


def vector_angle(vec1, vec2):
    """
    calculate the angle between two vectors
    :param vec1: vector as np.array()
    :param vec2: vector as np.array()
    :return: angle in degree
    """
    return np.degrees(np.arccos(np.dot(vec1, vec2).astype(np.float) / (norm(vec1) * norm(vec2))))


def prob_norm(vec):
    return vec.astype(np.float) / np.sum(vec) if np.sum(vec) > 0. else vec


def hand_eye_obj(joints, face_range=1, angle_range=40):  # pixel unit

    hand_obj_list = []
    eye_obj_list = []
    hand_eye = 0

    obj_list = rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes)

    # right face, left face length in pixel unit
    rface_len = norm(joints[16] - joints[0]) if np.all(joints[16] > 0) else -1
    lface_len = norm(joints[17] - joints[0]) if np.all(joints[17] > 0) else -1

    # right arm, left arm length in pixel unit
    rarm_len = norm(joints[4] - joints[3]) if (np.all(joints[4] > 0)) and (np.all(joints[3] > 0)) else -1
    larm_len = norm(joints[7] - joints[6]) if (np.all(joints[7] > 0)) and (np.all(joints[6] > 0)) else -1

    if rarm_len > 0:
        pose_range = rarm_len * 1.2
    elif larm_len > 0:
        pose_range = larm_len * 1.2
    else:
        pose_range = 60

    # check whether human is facing robot or not
    if rface_len >= 0 and lface_len >= 0 and np.abs(rface_len - lface_len) < face_range:
        print 'human facing robot'

    else:  # human not facing robot
        ear = joints[16] if rface_len > lface_len else joints[17]  # check right or left ear
        rhand_vec = joints[4] - ear if np.all(joints[4] > 0) else -1
        lhand_vec = joints[7] - ear if np.all(joints[7] > 0) else -1
        eye_vec = joints[0] - ear

        # check if eyes are looking at hands
        rhand_eye_angle = vector_angle(eye_vec, rhand_vec) if np.all(joints[4] > 0) else -1
        lhand_eye_angle = vector_angle(eye_vec, lhand_vec) if np.all(joints[7] > 0) else -1
        hand_eye = int(0 < rhand_eye_angle < angle_range or 0 < lhand_eye_angle < angle_range)  # 1 or 0

        for obj in obj_list.bounding_boxes:

            # check if human skeleton is in current bounding box
            if obj.Class == 'person' and isin_bbox(joints[0], obj.xmin, obj.xmax, obj.ymin, obj.ymax):
                continue

            # elif obj.Class == 'book':  # may be 'else' in the future
            x_mean = (obj.xmin + obj.xmax) / 2.0
            y_mean = (obj.ymin + obj.ymax) / 2.0
            obj_pos = np.array([x_mean, y_mean])

            # left_hand, right_hand distance to objects in pixel unit
            rhand_dis = norm(joints[4] - obj_pos) if np.all(joints[4] > 0) else -1
            lhand_dis = norm(joints[7] - obj_pos) if np.all(joints[7] > 0) else -1

            if (0 < lhand_dis < pose_range) or (0 < rhand_dis < pose_range):
                hand_obj_list.append(obj)

            # check if eyes are looking at object, calculate the angle btw object and eyes
            obj_vec = obj_pos - ear
            obj_eye_angle = vector_angle(eye_vec, obj_vec)

            # if __debug__:
            #     print 'eye obj angle = ', obj_eye_angle

            if np.abs(obj_eye_angle) < angle_range:  # unit: degree
                eye_obj_list.append(obj)

        # if __debug__:
            # print 'rhand_vec = ', rhand_vec
            # print 'lhand_vec = ', lhand_vec
            # print 'eye_vec   = ', eye_vec
            # print 'hand obj list = ', [ele.Class for ele in hand_obj_list]
            # print 'eye obj list  = ', [ele.Class for ele in eye_obj_list]
            # print 'hand eye angle = ', rhand_eye_angle, lhand_eye_angle
            # print 'looking at hand? ', hand_eye

    return hand_obj_list, eye_obj_list, hand_eye


def get_action(hand_obj_list, eye_obj_list, hand_eye):
    """
    probability distribution for action recognition based on hand, eye, objects
    :param hand_obj_list: list of objects around hands
    :param eye_obj_list: list of objects eyes look at
    :param hand_eye: true if human looks at hands
    :return: index with max action probability
    """
    p_acts_hand = np.zeros(action_num, np.float)  # shape = (action_num,)
    p_acts_eye = np.zeros(action_num, np.float)  # shape = (action_num,)

    for obj in hand_obj_list:
        p_acts_hand += prob_norm(hand_acts.loc[obj.Class, :].values) * obj.probability

    for obj in eye_obj_list:
        p_acts_eye += prob_norm(eyes_acts.loc[obj.Class, :].values) * obj.probability

    if np.sum(p_acts_hand) == 0.:
        p_acts_hand[-1] = 1.

    if np.sum(p_acts_eye) == 0.:
        p_acts_eye[-1] = 1.

    p_acts_hand = prob_norm(p_acts_hand)  # normalize the probability
    p_acts_eye = prob_norm(p_acts_eye)  # normalize the probability

    p_acts = prob_norm(p_acts_hand + p_acts_eye * 1.2 + p_eye_hand * hand_eye)
    act_id = np.argmax(p_acts) if np.max(p_acts) > 0.25 else -1

    # if __debug__:
    #     action = action_cat[act_id]
    #     print 'p(act|hand) = ', p_acts_hand
    #     print 'p(act|eyes) = ', p_acts_eye
    #     print 'p(action)   = ', p_acts
    #     print 'action = ', action

    return act_id


def person_callback(data):
    """
    :param data.image_w = 320, data.image_h = 480
    :return: action
    """

    if rospy.get_param('/thesis/action_on', False):
        try:
            img_stitch = cv_bridge.imgmsg_to_cv2(rospy.wait_for_message('/thesis/img_stitching', Image, timeout=10), "bgr8")

        except rospy.exceptions.ROSException:
            rospy.logerr("Error when fetching img_stitching.")
            return

        person_list = get_people_joints(data)

        for joints in person_list:
            # human identification
            human_result = identify_single_human(img_stitch, joints, human_info, None)

            hand_obj_list, eye_obj_list, hand_eye = hand_eye_obj(joints)
            action_id = get_action(hand_obj_list, eye_obj_list, hand_eye)
            print 'action_id = ', action_id

            if human_result is not None:
                # Add action to human
                if human_result.action != action_id:
                    human_result.action = int(action_id)

                # Add location to human if exists
                if rospy.has_param('/thesis/pepper_location'):  # type: int
                    human_result.location = rospy.get_param('/thesis/pepper_location')

                store_human_info(human_result)

    return


def get_action_cat():
    if rospy.has_param('action_cat'):
        return rospy.get_param('action_cat')
    else:
        h_acts = pd.read_csv(rospkg.RosPack().get_path('thesis') + '/config/hand_actions.csv', sep=',')  # DataFrame
        rospy.set_param('action_cat', h_acts.columns.to_list())
        return h_acts.columns.to_list()


if __name__ == '__main__':
    # global const for action recognition
    pkg_dir = rospkg.RosPack().get_path('thesis')
    config_dir = pkg_dir + '/config/'
    human_info_dir = rospkg.RosPack().get_path('thesis') + '/human_info/'

    hand_acts = pd.read_csv(config_dir + 'hand_actions.csv', sep=',')  # DataFrame
    eyes_acts = pd.read_csv(config_dir + 'eyes_actions.csv', sep=',')  # DataFrame
    eye_hand_acts = pd.read_csv(config_dir + 'eyes_hand.csv', sep=',')  # DataFrame
    p_eye_hand = prob_norm(eye_hand_acts.values[0])  # shape=(action_num,)

    action_cat = hand_acts.columns.to_list()  # category of actions
    action_num = len(action_cat)
    part_num = 18
    rospy.set_param('action_cat', action_cat)

    # for human identification
    cv_bridge = CvBridge()
    human_info = load_human_info(human_info_dir)

    rospy.init_node('action_recognition', log_level=rospy.INFO)
    rospy.loginfo('action_reg start!')

    rospy.Subscriber('/thesis/human_pose', Persons, person_callback, queue_size=1)
    rospy.spin()
