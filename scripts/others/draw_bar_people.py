#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt

if __name__ == '__main__':

    # draw num of figures
    tick_size = 20
    label_size = 22
    title_size = 25
    num_size = 15
    width = 0.5

    p_img = [479, 1116, 1042, 564, 633, 256, 234, 324, 325]

    plt.bar(np.arange(len(p_img)), p_img, width)
    plt.xticks(np.arange(len(p_img)), np.arange(len(p_img)).tolist(), fontsize=tick_size)
    plt.yticks(np.arange(0, 1200, 100), fontsize=num_size)
    plt.title('Frame Distribution w.r.t People', fontsize=title_size)
    plt.xlabel('Sample', fontsize=label_size)
    plt.ylabel('Number of frames', fontsize=label_size)

    plt.show()
