#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
Draw pyplot about simulation time
"""
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    num = [10, 20, 50, 80, 100, 150, 200]

    # total time of different planners
    fcfs = [345.9566667, 641.49, 1672.733333, 2662.676667, 3469.893333, 5194.183333, 7121.913333]
    rand = [344.3, 778.41, 1694.46, 2725.813333, 3496.766667, 5233.75, 6789.766667]
    pf = [320.46, 722.3933333, 1754.76, 2787.856667, 3325.553333, 5202.06, 6896.566667]
    sf = [158.0666667, 219.1733333, 418.69, 633.5933333, 755.9866667, 1133.556667, 1401.316667]
    dp = [159.3066667, 251.1833333, 435.56, 572.67, 658.3166667, 939.52, 1168.256667]

    f = plt.figure(num=None, figsize=(14, 9), dpi=80, facecolor='w', edgecolor='k')
    ax = f.add_subplot(111)
    ax.tick_params(labelright=False)

    mark_size = 18
    lw = 3.5
    num_size = 20
    label_size = 25
    title_size = 30
    legend_size = 20

    plt.plot(num, fcfs, linewidth=3.0, color='blue', marker='o', ms=mark_size, label='FCFS', zorder=0)
    plt.plot(num, rand, linewidth=3.0, color='purple', marker='D', ms=mark_size, label='Random', zorder=1)
    plt.plot(num, pf, linewidth=3.0, color='green', marker='P', ms=mark_size, label='PF', zorder=2)
    plt.plot(num, sf, linewidth=3.0, color='orange', marker='X', ms=mark_size, label='SF', zorder=4)
    plt.plot(num, dp, linewidth=3.0, color='red', marker='s', ms=mark_size, label='Ours', zorder=3)

    plt.yticks(np.arange(0, 7200, 500), fontsize=num_size)
    plt.xticks(num, fontsize=num_size)
    plt.xlabel('Number of instructions', fontsize=label_size)
    plt.ylabel('Processing time (sec)', fontsize=label_size)
    plt.title('Total Processing Time', fontsize=title_size)
    plt.grid(True)

    box = ax.get_position()
    ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])

    params = {'legend.fontsize': legend_size,
              'legend.handlelength': 2}
    plt.rcParams.update(params)
    ax.legend(loc='center left', bbox_to_anchor=(1.0, 0.1))

    plt.show()
