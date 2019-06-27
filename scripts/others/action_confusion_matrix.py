#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import pandas as pd
import numpy as np
import os


if __name__ == '__main__':
    _video_dir = '/home/robot/pepper_data/action_images/'

    gt_df = pd.read_csv('/home/robot/pepper_data/result/gt/S000P000A006.txt', sep=" ", header=None)
    print gt_df

    # for v_name in os.listdir(_video_dir):
    #     pred_df = pd.read_csv('/home/robot/pepper_data/result/pred/' + v_name + '_action.csv')
    #     gt_df = pd.read_csv('/home/robot/pepper_data/result/gt/' + v_name + '.txt', sep=" ", header=None)
    #
    #     pred_dict = dict()
    #     gt_dict = dict()
    #     for i in range(len(pred_df)):
    #         pred_dict['frame'][i] = pred_df['action'][i]
