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

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Check roslaunch arg')
    parser.add_argument('--max_num', type=int, default=10)
    parser.add_argument('--seed', type=int, default=1000)
    args = parser.parse_args(rospy.myargv()[1:])

    _exp_dir = rospkg.RosPack().get_path('thesis') + '/exp2/instr_' + str(args.max_num) + '_' + str(args.seed) + '/'

    planners = ['opt', 'fcfs', 'rand', 'pf', 'sf', 'dp']

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
