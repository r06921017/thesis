#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import os

if __name__ == '__main__':
    gt_files = os.listdir('/home/robot/pepper_data/result/gt')
    pred_files = os.listdir('/home/robot/pepper_data/result/pred')

    for g in sorted(gt_files):
        # print p.split('.')[0].split('_')[0] + '.txt'
        if g.split('.')[0] + '_action.csv' not in pred_files:
            print g
