#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt

if __name__ == '__main__':
    _video_dir = '/home/robot/pepper_data/action_images/'
    _function_dict = {'eat': 0, 'drink': 1, 'television': 2, 'play': 3, 'chat': 4, 'call': 5, 'read': 6, 'work': 7,
                      'store': 8, 'dress': 9, 'sleep': 10, 'go_out': 11, 'wash': 12, 'other': 13}

    classes = ['eat', 'drink', 'television', 'chat', 'call', 'read', 'work',
               'store', 'sleep', 'go_out', 'wash', 'other']

    _video_list = os.listdir(_video_dir)
    # _video_list = ['S000P001A006']

    # gt_df = pd.read_csv('/home/robot/pepper_data/result/gt/S000P000A006.txt', sep=" ", header=None)
    # print type(gt_df[0][0])

    accu_frame = 0
    total_frame = 0

    action_matrix = np.zeros((len(_function_dict), len(_function_dict)))
    gt_vector = np.zeros(len(_function_dict))
    gt_all_vec = np.zeros(len(_function_dict))

    for v_name in _video_list:
        print 'v_name: ', v_name

        if v_name + '.txt' in os.listdir('/home/robot/pepper_data/result/gt/'):

            gt_df = pd.read_csv('/home/robot/pepper_data/result/gt/' + v_name + '.txt', sep=" ", header=None)

            for i in range(len(gt_df)):
                gt_all_vec[_function_dict[gt_df[1][i]]] += 1

    print gt_all_vec
    print np.sum(gt_all_vec)

    show_class = ['eat', 'drink', 'watch TV', 'chat', 'call', 'read', 'work',
               'store', 'sleep', 'go out', 'wash', 'other']

    # draw num of figures
    tick_size = 20
    label_size = 22
    title_size = 25
    num_size = 15
    width = 0.5
    plt.bar(np.arange(len(show_class)), np.delete(gt_all_vec, [3, 9]).tolist(), width)
    plt.xticks(np.arange(len(show_class)), show_class, fontsize=tick_size)
    plt.yticks(np.arange(0, 600, 50), fontsize=num_size)
    plt.title('Frame Distribution w.r.t Actions', fontsize=title_size)
    plt.xlabel('Actions', fontsize=label_size)
    plt.ylabel('Number of frames', fontsize=label_size)

    plt.show()
