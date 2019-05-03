#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('test_node_viz', anonymous=True, log_level=rospy.INFO)
    pub = rospy.Publisher('/thesis/robot_node', String, queue_size=10)
    temp_list = ['2', '23_1', '23_2', '23_3', '3', '03_8', '03_7', '03_6', '03_5', '03_4', '03_3', '03_2', '03_1', '0']
    i = 0
    period = 0.5  # second
    rate = rospy.Rate(1.0 / period)
    while not rospy.is_shutdown():
        if i >= len(temp_list):
            break

        temp = String()
        temp.data = temp_list[i]
        print 'temp_list = ', temp
        pub.publish(temp)
        i += 1
        rate.sleep()
