#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospkg
import pandas as pd
import numpy as np

if __name__ == '__main__':
    pkg_dir = rospkg.RosPack().get_path('thesis')
    pose_df = pd.read_csv(pkg_dir+'/exp2/position.csv')
    x_list = pose_df['x']
    y_list = pose_df['y']

    lose = 0.0
    for i in range(len(x_list)):
        x_lose1 = abs(x_list[i] - 1.5)
        x_lose2 = abs(x_list[i] - 2.7)
        y_lose1 = abs(y_list[i] - 0.8)
        y_lose2 = abs(y_list[i] + 0.4)

        lose += min(x_lose1, x_lose2, y_lose1, y_lose2)

    lose /= len(x_list)
    print 'avg lose = ', lose

    lose = list()
    for i in range(len(x_list)):
        x_lose1 = abs(x_list[i] - 1.5)
        x_lose2 = abs(x_list[i] - 2.7)
        y_lose1 = abs(y_list[i] - 0.8)
        y_lose2 = abs(y_list[i] + 0.4)

        lose.append(min(x_lose1, x_lose2, y_lose1, y_lose2))
        # if min(x_lose1, x_lose2, y_lose1, y_lose2) > lose:
        #     lose = min(x_lose1, x_lose2, y_lose1, y_lose2)

    # print 'max lose = ', lose

    print np.mean(lose)
    print np.std(lose)
    print max(lose)
