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
    # fcfs = [345.9566667, 641.49, 1672.733333, 2662.676667, 3469.893333, 5194.183333, 7121.913333]
    # rand = [344.3, 778.41, 1694.46, 2725.813333, 3496.766667, 5233.75, 6789.766667]
    # pf = [320.46, 722.3933333, 1754.76, 2787.856667, 3325.553333, 5202.06, 6896.566667]
    # sf = [158.0666667, 219.1733333, 418.69, 633.5933333, 755.9866667, 1133.556667, 1401.316667]
    # dp = [159.3066667, 251.1833333, 435.56, 572.67, 658.3166667, 939.52, 1168.256667]

    fcfs = [0.1630328544, 0.1431774729, 0.1511869734, 0.1448977575, 0.1468923415, 0.1466560888, 0.1414106164]
    rand = [0.1568399651, 0.1400290335, 0.1534412143, 0.1467995177, 0.1459262532, 0.1362260957, 0.147736336]
    pf = [0.1669830091, 0.13472983, 0.1386068314, 0.1534352285, 0.1575112641, 0.1422148464, 0.1444202492]
    sf = [0.327673714, 0.4076543253, 0.5776717434, 0.6320762491, 0.6529622864, 0.6851263069, 0.691236642]
    dp = [0.2949133982, 0.4094846073, 0.5878398172, 0.7077395415, 0.7547935757, 0.8047040699, 0.8754247444]

    f = plt.figure(num=None, figsize=(14, 9), dpi=80, facecolor='w', edgecolor='k')
    ax = f.add_subplot(111)
    ax.tick_params(labelright=True)

    plt.plot(num, fcfs, linewidth=4.0, color='blue', marker='s', ms=14, label='FCFS')
    plt.plot(num, rand, linewidth=4.0, color='purple', marker='s', ms=14, label='Random')
    plt.plot(num, pf, linewidth=4.0, color='green', marker='s', ms=14, label='PF')
    plt.plot(num, sf, linewidth=4.0, color='orange', marker='s', ms=14, label='SF')
    plt.plot(num, dp, linewidth=4.0, color='red', marker='s', ms=14, label='Ours')

    # plt.yticks(np.arange(150, 7200, 500), fontsize=16)
    plt.yticks(np.arange(0, 1.05, 0.1), fontsize=16)
    plt.xticks(num, fontsize=16)
    plt.xlabel('Number of instructions', fontsize=16)
    # plt.ylabel('Processing time (sec)', fontsize=16)
    plt.ylabel('Time ratio', fontsize=16)
    plt.title('Time Ratio of Planners', fontsize=20)

    box = ax.get_position()
    ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])

    params = {'legend.fontsize': 16,
              'legend.handlelength': 2}
    plt.rcParams.update(params)
    ax.legend(loc='center left', bbox_to_anchor=(1.1, 0.1))

    plt.show()
