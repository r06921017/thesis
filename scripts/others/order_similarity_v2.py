#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
Calculate the similarity of scheduling order
"""
import rospy
import argparse
import numpy as np
import pandas as pd
import rospkg
import os

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Check roslaunch arg')
    parser.add_argument('--max_num', type=int, default=10)
    parser.add_argument('--seed', type=int, default=1000)
    args = parser.parse_args(rospy.myargv()[1:])

    # _exp_dir = rospkg.RosPack().get_path('thesis') + '/exp2/instr_' + str(args.max_num) + '_' + str(args.seed) + '/'

    _exp2 = rospkg.RosPack().get_path('thesis') + '/exp2'
    # _exp_dir_list = os.listdir(_exp2)
    # print _exp_dir_list

    _exp_dir_list = ['instr_10_1002', 'instr_10_1005', 'instr_10_1004', 'instr_10_1001', 'instr_10_1111']

    planners = ['opt', 'fcfs', 'rand', 'pf', 'sf', 'dp']

    total_cost = np.zeros((len(_exp_dir_list), len(planners)-1))

    for exp_idx, exp_d in enumerate(_exp_dir_list):
        _exp_dir = _exp2 + '/' + exp_d + '/'
        seq = list()

        for p in planners:
            temp_df = pd.read_csv(_exp_dir + 'task_motion_planner_' + p + '_sim_done.csv')
            temp_dict = dict()

            print p, temp_df['done'].tolist()

            for i, e in enumerate(temp_df['done'].tolist()):
                temp_dict[e] = i  # {instr_id, index}

            seq.append(temp_dict)

        cost = np.zeros(len(seq)-1)
        for i in range(1, len(seq)):
            for e in seq[0].keys():
                cost[i-1] += abs(seq[i][e] - seq[0][e])

        print 'cost: ', cost
        total_cost[exp_idx, :] = cost

    print 'total_cost'
    print total_cost

    print 'mean cost'
    print np.mean(total_cost, axis=0)
