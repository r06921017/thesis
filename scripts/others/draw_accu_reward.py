#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
Draw pyplot about accumulative reward
"""
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rospkg

if __name__ == '__main__':
    _exp_dir = rospkg.RosPack().get_path('thesis')+'/experiments/'
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

    temp_df = pd.read_csv(_exp_dir + 'task_motion_planner_opt_sim_opt_reward.csv')
    opt_time = temp_df['time'].tolist()
    opt_reward = temp_df['reward'].tolist()

    temp_df = pd.read_csv(_exp_dir + 'task_motion_planner_opt_sim_reward.csv')
    opt_real_time = temp_df['time'].tolist()
    opt_real_reward = temp_df['reward'].tolist()

    f = plt.figure(num=None, figsize=(14, 9), dpi=80, facecolor='w', edgecolor='k')
    ax = f.add_subplot(111)
    ax.tick_params(labelright=True)

    plt.plot(opt_time, opt_reward, linewidth=3.0, color='deeppink', marker='*', ms=15, label='Opt', zorder=0)
    plt.plot(opt_real_time, opt_real_reward, linewidth=3.0, color='pink', marker='v', ms=14, label='Opt w/p', zorder=1)
    plt.plot(fcfs_time, fcfs_reward, linewidth=3.0, color='blue', marker='o', ms=14, label='FCFS', zorder=2)
    plt.plot(rand_time, rand_reward, linewidth=3.0, color='purple', marker='D', ms=14, label='Random', zorder=3)
    plt.plot(pf_time, pf_reward, linewidth=3.0, color='green', marker='P', ms=14, label='PF', zorder=4)
    plt.plot(sf_time, sf_reward, linewidth=3.0, color='orange', marker='X', ms=14, label='SF', zorder=6)
    plt.plot(dp_time, dp_reward, linewidth=3.0, color='red', marker='s', ms=14, label='Ours', zorder=5)

    max_step = np.ceil(np.max([fcfs_time[-1], opt_time[-1], opt_real_time[-1]])).astype(int)
    step_gap = 10
    num = range(0, max_step+step_gap, step_gap)

    reward_bound = np.ceil(opt_reward[-1])
    reward_step = 0.25
    plt.yticks(np.arange(0, reward_bound, 0.25), fontsize=16)
    plt.xticks(num, fontsize=16)
    plt.xlabel('Time step', fontsize=20)
    plt.ylabel('Reward value', fontsize=20)
    plt.title('Accumulative Reward', fontsize=30)
    plt.grid(True)

    box = ax.get_position()
    ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])

    params = {'legend.fontsize': 16,
              'legend.handlelength': 2}
    plt.rcParams.update(params)
    ax.legend(loc='center left', bbox_to_anchor=(1.1, 0.1))

    plt.show()
