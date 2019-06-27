#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
Draw pyplot about accumulative reward
"""
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rospy
import rospkg
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Check roslaunch arg')
    parser.add_argument('--max_num', type=int, default=10)
    parser.add_argument('--seed', type=int, default=1000)
    args = parser.parse_args(rospy.myargv()[1:])

    _exp_dir = rospkg.RosPack().get_path('thesis') + '/exp2/instr_' + str(args.max_num) + '_' + str(args.seed) + '/'
    temp_df = pd.read_csv(_exp_dir + 'task_motion_planner_fcfs_sim_reward.csv')
    fcfs_time = temp_df['time'].tolist()
    fcfs_reward = temp_df['reward'].tolist()

    temp_df = pd.read_csv(_exp_dir + 'task_motion_planner_rand_sim_reward.csv')
    rand_time = temp_df['time'].tolist()
    rand_reward = temp_df['reward'].tolist()

    temp_df = pd.read_csv(_exp_dir + 'task_motion_planner_pf_sim_reward.csv')
    pf_time = temp_df['time'].tolist()
    pf_reward = temp_df['reward'].tolist()

    temp_df = pd.read_csv(_exp_dir + 'task_motion_planner_sf_sim_reward.csv')
    sf_time = temp_df['time'].tolist()
    sf_reward = temp_df['reward'].tolist()

    temp_df = pd.read_csv(_exp_dir + 'task_motion_planner_dp_sim_reward.csv')
    dp_time = temp_df['time'].tolist()
    dp_reward = temp_df['reward'].tolist()

    temp_df = pd.read_csv(_exp_dir + 'opt_reward.csv')
    opt_time = temp_df['time'].tolist()
    opt_reward = temp_df['reward'].tolist()

    temp_df = pd.read_csv(_exp_dir + 'task_motion_planner_opt_sim_reward.csv')
    opt_real_time = temp_df['time'].tolist()
    opt_real_reward = temp_df['reward'].tolist()

    f = plt.figure(num=None, figsize=(14, 9), dpi=80, facecolor='w', edgecolor='k')
    ax = f.add_subplot(111)
    ax.tick_params(labelright=True)

    mark_size = 18
    lw = 3.5
    num_size = 20
    label_size = 25
    title_size = 30
    legend_size = 20

    plt.plot(opt_time, opt_reward, linewidth=lw, color='deeppink', marker='*', ms=15, label='Opt', zorder=0)
    plt.plot(opt_real_time, opt_real_reward, linewidth=lw, color='grey', marker='v', ms=14, label='Opt w/m', zorder=1)
    plt.plot(fcfs_time, fcfs_reward, linewidth=lw, color='blue', marker='o', ms=14, label='FCFS', zorder=2)
    plt.plot(rand_time, rand_reward, linewidth=lw, color='purple', marker='D', ms=14, label='Random', zorder=3)
    plt.plot(pf_time, pf_reward, linewidth=lw, color='green', marker='P', ms=14, label='PF', zorder=4)
    plt.plot(sf_time, sf_reward, linewidth=lw, color='orange', marker='X', ms=14, label='SF', zorder=6)
    plt.plot(dp_time, dp_reward, linewidth=lw, color='red', marker='s', ms=14, label='Ours', zorder=5)

    max_step = np.ceil(np.max([fcfs_time[-1], opt_time[-1], opt_real_time[-1]])).astype(int)
    step_gap = 10
    num = range(0, max_step+step_gap, step_gap)

    reward_bound = np.ceil(opt_reward[-1])
    reward_step = 0.5
    plt.yticks(np.arange(0, reward_bound+reward_step, reward_step), fontsize=num_size)
    plt.xticks(num, fontsize=num_size)
    plt.xlabel('Time step', fontsize=label_size)
    plt.ylabel('Reward value', fontsize=label_size)
    plt.title('Accumulative Reward', fontsize=title_size)
    plt.grid(True)

    box = ax.get_position()
    ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])

    params = {'legend.fontsize': legend_size,
              'legend.handlelength': 2}
    plt.rcParams.update(params)
    ax.legend(loc='center left', bbox_to_anchor=(1.1, 0.1))

    plt.show()
