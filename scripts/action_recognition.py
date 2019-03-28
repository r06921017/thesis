#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This is for human action recognition
"""
import rospy
import numpy as np
import pandas as pd
from numpy.linalg import norm
from tfpose_ros.msg import Persons
from darknet_ros_msgs.msg import *


def isin_bbox(pt, xmin, xmax, ymin, ymax):
    """
    :param pt: np.array([x, y])
    :param xmin: int
    :param xmax: int
    :param ymin: int
    :param ymax: int
    :return: bool
    """
    ll = np.array([xmin, ymin])  # lower-left
    ur = np.array([xmax, ymax])  # upper-right

    return np.all(np.logical_and(ll <= pt, pt <= ur))


def get_vector_angle(vec1, vec2):
    """
    calculate the angle between two vectors
    :param vec1: vector as np.array()
    :param vec2: vector as np.array()
    :return: angle in degree
    """
    return np.arccos(np.dot(vec1, vec2) / (norm(vec1) * norm(vec2))) / np.pi * 180


def get_hand_eye_obj(joints, pose_range=60, face_range=1, angle_range=55):  # pixel unit

    hand_obj_list = []
    eye_obj_list = []
    hand_eye = None

    obj_list = rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes)

    for obj in obj_list.bounding_boxes:

        # check if human skeleton is in current bounding box
        if obj.Class == 'person' and isin_bbox(joints[0], obj.xmin, obj.xmax, obj.ymin, obj.ymax):
            continue

        elif obj.Class == 'book':  # may be 'else' in the future
            x_mean = (obj.xmin + obj.xmax) / 2.0
            y_mean = (obj.ymin + obj.ymax) / 2.0
            obj_pos = np.array([x_mean, y_mean])

            # left_hand, right_hand distance to objects in pixel unit
            rhand_dis = norm(joints[4] - obj_pos) if np.all(joints[4] > 0) else -1
            lhand_dis = norm(joints[7] - obj_pos) if np.all(joints[7] > 0) else -1

            if (0 < lhand_dis < pose_range) or (0 < rhand_dis < pose_range):
                hand_obj_list.append(obj)

            # right face, left face length in pixel unit
            rface_len = norm(joints[16] - joints[0]) if np.all(joints[16] > 0) else -1
            lface_len = norm(joints[17] - joints[0]) if np.all(joints[17] > 0) else -1
            print 'rface_len, lface_len = ', rface_len, ', ', lface_len

            # check whether human is facing robot or not
            if rface_len >= 0 and lface_len >= 0 and np.abs(rface_len-lface_len) < face_range:
                print 'human facing robot'

            else:  # human not facing robot
                if rface_len > lface_len:
                    rhand_vec = joints[4] - joints[16] if np.all(joints[4] > 0) else -1
                    lhand_vec = joints[7] - joints[16] if np.all(joints[7] > 0) else -1
                    eye_vec = joints[0] - joints[16]
                    obj_vec = obj_pos - joints[16]
                else:
                    rhand_vec = joints[4] - joints[17] if np.all(joints[4] > 0) else -1
                    lhand_vec = joints[7] - joints[17] if np.all(joints[7] > 0) else -1
                    eye_vec = joints[0] - joints[17]
                    obj_vec = obj_pos - joints[17]

                # check if eyes are looking at object, calculate the angle btw object and eyes
                obj_eye_angle = get_vector_angle(eye_vec, obj_vec)
                if np.abs(obj_eye_angle) < angle_range:  # unit: degree
                    eye_obj_list.append(obj)

                # check if eyes are looking at hands
                rhand_eye_angle = get_vector_angle(eye_vec, rhand_vec) if np.all(joints[4] > 0) else -1
                lhand_eye_angle = get_vector_angle(eye_vec, lhand_vec) if np.all(joints[7] > 0) else -1

                print 'hand eye angle = ', rhand_eye_angle, lhand_eye_angle

                hand_eye = (0 < rhand_eye_angle < angle_range or 0 < lhand_eye_angle < angle_range)  # bool

    return hand_obj_list, eye_obj_list, hand_eye


def action_recognition(hand_obj_list, eye_obj_list, hand_eye):

    def prob_norm(vec):
        return vec.astype(np.float) / np.sum(vec)

    action_prob = np.zeros(action_num, np.float)  # shape = (14,)

    for obj in hand_obj_list:
        hand_prob = prob_norm(hand_actions[obj, :].values)
        eyes_prob = prob_norm(eyes_actions[obj, :].values)
        heas_prob = prob_norm(eyes_hand[obj, :].values)  # hand-eyes-actions

    return


def person_callback(data):
    """
    :param data.image_w = 320, data.image_h = 480
    :return: action
    """
    person_list = []  # a list of 2D array
    head_chest_range = 40  # distance threshold btw joint[0] and joint[1]

    for idx, person in enumerate(data.persons):
        joints = np.ones((part_num, 2), dtype=np.int) * -1
        for i in range(len(data.persons[idx].body_part)):
            part = data.persons[0].body_part[i]
            # Transform the joint points back to the position on the image
            joints[part.part_id, 0] = part.x * data.image_w
            joints[part.part_id, 1] = part.y * data.image_h

        # filtering person who is too far or no head
        if np.all(joints[0] > 0) and (np.all(joints[16] > 0) or np.all(joints[17] > 0)):  # person has nose and one ear
            if np.all(joints[1] > 0):  # person has chest
                if np.linalg.norm((joints[0] - joints[1])) > head_chest_range:  # filtering person who is too far
                    person_list.append(joints)

            elif np.all(joints[2] > 0):
                if np.linalg.norm((joints[0] - joints[2])) > head_chest_range:  # filtering person who is too far
                    person_list.append(joints)

            elif np.all(joints[5] > 0):
                if np.linalg.norm((joints[0] - joints[5])) > head_chest_range:  # filtering person who is too far
                    person_list.append(joints)

    for idx, joints in enumerate(person_list):
        hand_obj_list, eye_obj_list, hand_eye = get_hand_eye_obj(joints)

        if __debug__:
            print 'person:', idx, ' hand:',
            for temp_obj in hand_obj_list:
                print temp_obj.Class, '(', temp_obj.probability, '), ',
            print '| head:',
            for temp_obj in eye_obj_list:
                print temp_obj.Class,
            print ' '
            print 'looking at hand? ', hand_eye
    return


if __name__ == '__main__':
    # global const
    hand_actions = pd.read_csv('../config/hand_actions.csv', sep=',')  # DataFrame
    eyes_actions = pd.read_csv('../config/eyes_actions.csv', sep=',')  # DataFrame
    eyes_hand = pd.read_csv('../config/eyes_hand.csv', sep=',')  # DataFrame
    
    action_cat = hand_actions.columns.to_list()  # category of actions
    action_num = len(action_cat)
    w = 320
    h = 480
    part_num = 18

    rospy.init_node('action_reg', log_level=rospy.INFO)
    rospy.loginfo('action_reg start!')

    rospy.Subscriber('/thesis/human_pose', Persons, person_callback, queue_size=1)
    rospy.spin()
