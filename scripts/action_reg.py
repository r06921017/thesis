#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This is for human action recognition
"""
import rospy
import numpy as np
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


def get_hand_head_obj(person_joints, pose_range=60, face_range=1, angle_range=45):  # pixel unit

    hand_obj_list = []
    head_obj_list = []
    obj_list = rospy.wait_for_message('/darknet_ros/bounding_boxes', BoundingBoxes)

    for obj in obj_list.bounding_boxes:

        if obj.Class == 'person' and isin_bbox(person_joints[0], obj.xmin, obj.xmax, obj.ymin, obj.ymax):
            continue

        elif obj.Class == 'book':
            x_mean = (obj.xmin + obj.xmax) / 2.0
            y_mean = (obj.ymin + obj.ymax) / 2.0
            obj_pos = np.array([x_mean, y_mean])

            # lh, rh: left_hand, right_hand distance in pixel unit
            rh_dis = norm(person_joints[4] - obj_pos) if np.all(person_joints[4] > 0) else -1
            lh_dis = norm(person_joints[7] - obj_pos) if np.all(person_joints[7] > 0) else -1

            if (0 < lh_dis < pose_range) or (0 < rh_dis < pose_range):
                hand_obj_list.append(obj)

            # objects in human sight
            # lf, rf: left_face, right_face distance in pixel unit
            rf_dis = norm(person_joints[16] - person_joints[0]) if np.all(person_joints[16] > 0) else -1
            lf_dis = norm(person_joints[17] - person_joints[0]) if np.all(person_joints[17] > 0) else -1
            print 'rf_dis, lf_dis = ', rf_dis, ', ', lf_dis

            if rf_dis >= 0 and lf_dis >= 0 and np.abs(rf_dis-lf_dis) < face_range:
                print 'human facing robot'

            else:
                face_vec = person_joints[0]-person_joints[16] if rf_dis > lf_dis else person_joints[0]-person_joints[17]
                obj_vec = obj_pos-person_joints[16] if rf_dis > lf_dis else obj_pos-person_joints[17]

                obj_face_angle = np.arccos(np.dot(face_vec, obj_vec) / (norm(face_vec)*norm(obj_vec))) / np.pi * 180
                # print 'obj_face_angle = ', obj_face_angle

                if np.abs(obj_face_angle) < angle_range:
                    head_obj_list.append(obj)

    return hand_obj_list, head_obj_list


def person_callback(data):
    """
    :param data.image_w = 320, data.image_h = 480
    :return: action
    """
    person_list = []  # a list of 2D array
    head_chest_range = 40

    for idx, person in enumerate(data.persons):
        person_joints = np.ones((part_num, 2), dtype=np.int) * -1
        for i in range(len(data.persons[idx].body_part)):
            part = data.persons[0].body_part[i]
            # Transform the joint points back to the position on the image
            person_joints[part.part_id, 0] = part.x * data.image_w
            person_joints[part.part_id, 1] = part.y * data.image_h

        # Filtering person who is too far
        if np.all(person_joints[0] != -1):  # person has nose
            if np.all(person_joints[1] != -1):  # person has chest
                if np.linalg.norm((person_joints[0] - person_joints[1])) > head_chest_range:
                    person_list.append(person_joints)

            elif np.all(person_joints[2] != -1):
                if np.linalg.norm((person_joints[0] - person_joints[2])) > head_chest_range:
                    person_list.append(person_joints)

            elif np.all(person_joints[5] != -1):
                if np.linalg.norm((person_joints[0] - person_joints[5])) > head_chest_range:
                    person_list.append(person_joints)

    for idx, person_joints in enumerate(person_list):
        hand_obj_list, head_obj_list = get_hand_head_obj(person_joints)

        print 'person:', idx, ' hand:',
        for temp_obj in hand_obj_list:
            print temp_obj.Class, '(', temp_obj.probability, '), ',
        print '| head:',
        for temp_obj in head_obj_list:
            print temp_obj.Class,
        print ' '
    return


if __name__ == '__main__':
    rospy.init_node('action_reg', log_level=rospy.INFO)
    w = 320
    h = 480
    part_num = 18
    # person_list = []
    rospy.loginfo('action_reg start!')

    rospy.Subscriber('/thesis/human_pose', Persons, person_callback, queue_size=1)
    rospy.spin()
