#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
transfer List of Human() into dictionary structure and set to ros_param
"""
from action_recognition import *


def load_human_info2dict(human_info_dir):
    h_list = load_human_info(human_info_dir)
    print 'h_list ==='
    print h_list
    rospy.set_param('human_list', h_list)
    print 'set list to param'

    name_dict = dict()
    ip_dict = dict()
    human_dict = dict()
    for h in h_list:
        name_dict[h.name] = h
        ip_dict[h.ip] = h

    human_dict['name'] = name_dict
    human_dict['ip'] = ip_dict

    return human_dict


if __name__ == '__main__':
    rospy.init_node('test')
    h_info_dir = rospkg.RosPack().get_path('thesis') + '/human_info/'
    temp_dict = load_human_info2dict(h_info_dir)
