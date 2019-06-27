#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import pandas as pd
import numpy as np
import os


if __name__ == '__main__':
    _video_dir = '/home/robot/pepper_data/action_images/'
    _function_dict = {'eat': 0, 'drink': 1, 'television': 2, 'play': 3, 'chat': 4, 'call': 5, 'read': 6, 'work': 7,
                      'store': 8, 'dress': 9, 'sleep': 10, 'go_out': 11, 'wash': 12, 'other': 13}

    _video_list = os.listdir(_video_dir)
    # _video_list = ['S000P001A006']

    gt_df = pd.read_csv('/home/robot/pepper_data/result/gt/S000P000A006.txt', sep=" ", header=None)
    print type(gt_df[0][0])

    accu_frame = 0
    total_frame = 0

    action_matrix = np.zeros((len(_function_dict), len(_function_dict)))
    gt_vector = np.zeros(len(_function_dict))

    for v_name in _video_list:
        pred_df = pd.read_csv('/home/robot/pepper_data/result/pred/' + v_name + '_action.csv')
        gt_df = pd.read_csv('/home/robot/pepper_data/result/gt/' + v_name + '.txt', sep=" ", header=None)

        pred_dict = dict()
        gt_dict = dict()
        for i in range(len(pred_df)):
            if pred_df['action'][i] == -1:
                val = 13
            else:
                val = pred_df['action'][i]

            pred_dict[int(pred_df['frame'][i])] = val

        for k in pred_dict.keys():
            gt_dict[k] = _function_dict[gt_df[1][k]]

        for k in pred_dict.keys():
            if pred_dict[k] == gt_dict[k]:
                accu_frame += 1

            action_matrix[int(pred_dict[k])][int(gt_dict[k])] += 1
            gt_vector[gt_dict[k]] += 1
            total_frame += 1

    print action_matrix
    print gt_vector
    print total_frame

    print 'confusion matrix'
    confusion_matrix = np.nan_to_num(action_matrix / gt_vector[None, :])
    np.set_printoptions(precision=2)
    print confusion_matrix
