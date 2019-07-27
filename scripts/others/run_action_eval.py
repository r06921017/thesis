#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import subprocess
import os
import time

if __name__ == '__main__':
    image_dir = '/home/robot/pepper_data/action_images/'
    pred_dir = '/home/robot/pepper_data/result/pred/'
    fps = 4.0

    for v_name in sorted(os.listdir(image_dir)):
        if v_name + '_action.csv' not in os.listdir(pred_dir):
            print v_name

            img_num = len(os.listdir(image_dir+v_name))
            print img_num

            a_str = 'gnome-terminal -x sh -c "rosrun thesis action_recognition.py --eval 1"'
            subprocess.call(a_str, shell=True)
            time.sleep(3)

            v_str = 'gnome-terminal -x sh -c "rosrun thesis test_ros_video.py --function 4 --video_name ' + v_name + '"'
            subprocess.call(v_str, shell=True)
            time.sleep(img_num / fps + 5)

            subprocess.call('rosnode kill action_recognition', shell=True)
            time.sleep(2)
