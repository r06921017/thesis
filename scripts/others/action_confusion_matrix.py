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

        if v_name + '_action.csv' in os.listdir('/home/robot/pepper_data/result/pred/') and\
                v_name + '.txt' in os.listdir('/home/robot/pepper_data/result/gt/'):
            pred_df = pd.read_csv('/home/robot/pepper_data/result/pred/' + v_name + '_action.csv')
            gt_df = pd.read_csv('/home/robot/pepper_data/result/gt/' + v_name + '.txt', sep=" ", header=None)

            pred_dict = dict()  # {'frame': 'action'}
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

            for i in range(len(gt_df)):
                gt_all_vec[_function_dict[gt_df[1][i]]] += 1

    confusion_matrix = np.nan_to_num(action_matrix / gt_vector[None, :])  # normalization
    cm = np.delete(confusion_matrix, (_function_dict['play'], _function_dict['dress']), axis=0)
    cm = np.delete(cm, (_function_dict['play'], _function_dict['dress']), axis=1)
    np.set_printoptions(precision=2)
    print cm
    print '------------------------------------------------------------'
    print gt_all_vec
    print np.sum(gt_all_vec)

    show_class = ['eat', 'drink', 'watch TV', 'chat', 'call', 'read', 'work',
               'store', 'sleep', 'go out', 'wash', 'other']

    label_size = 18
    title_size = 25

    # Draw with sklearn
    fig, ax = plt.subplots()
    im = ax.imshow(cm, interpolation='nearest', cmap='Greys')
    ax.figure.colorbar(im, ax=ax)

    # We want to show all ticks..
    ax.set(xticks=np.arange(cm.shape[1]),
           yticks=np.arange(cm.shape[0]),
           xticklabels=show_class, yticklabels=show_class)

    ax.set_title('Normalized confusion matrix', fontsize=title_size)
    ax.set_ylabel('Predicted label', fontsize=label_size)
    ax.set_xlabel('True label', fontsize=label_size)

    plt.title('Normalized confusion matrix', fontsize=title_size)
    plt.xlabel('True label', fontsize=label_size)
    plt.ylabel('Predicted label', fontsize=label_size)

    # Loop over data dimensions and create text annotations.
    fmt = '.2f'
    thresh = cm.max() / 2.
    for i in range(cm.shape[0]):
        for j in range(cm.shape[1]):
            ax.text(j, i, format(cm[i, j], fmt),
                    ha="center", va="center",
                    color="white" if cm[i, j] > thresh else "black")
    fig.tight_layout()

    # draw num of figures
    # label_size = 15
    # title_size = 25
    # num_size = 15
    # width = 0.5
    # plt.bar(np.arange(len(show_class)), np.delete(gt_all_vec, [3, 9]).tolist(), width)
    # plt.xticks(np.arange(len(show_class)), show_class, fontsize=label_size)
    # plt.yticks(np.arange(0, 550, 50), fontsize=num_size)

    plt.show()
