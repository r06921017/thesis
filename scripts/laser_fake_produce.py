#!/usr/bin/env python
# -*- coding: utf-8 -*-

# this is the file used to produce fake laser data to help Pepper's sparse laser data,
# also publish the tf from Tibia to laser

import sys 
import rospy
from sensor_msgs.msg import LaserScan
import tf
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')


def laser_callback(data):
    laser = LaserScan() 
    
    for i in range(len(data.ranges)):
        if i in range(15, 23):
            laser.ranges.append((data.ranges[23] - data.ranges[14])/9.0 * (i-14) + data.ranges[14])
        
        elif i in range(38, 46):
            laser.ranges.append((data.ranges[46] - data.ranges[37])/9.0 * (i-37) + data.ranges[37])
            
        else:
            laser.ranges.append(data.ranges[i])
        
    print laser.ranges

    laser.header = data.header
    laser.header.frame_id = 'laser'
    laser.angle_min = data.angle_min

    laser.angle_max = data.angle_max
    laser.angle_increment = data.angle_increment
    laser.time_increment = data.time_increment
    laser.scan_time = data.scan_time
    laser.range_min = data.range_min
    laser.range_max = data.range_max

    laser_pub.publish(laser)

    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0),
                     (0, 0, 0, 1),
                     rospy.Time.now(),
                     'laser',
                     "Tibia")
    return


if __name__ == '__main__':

    rospy.init_node('fake_laser_pepper', anonymous=True)
    rospy.Subscriber("/naoqi_driver_node/laser", LaserScan, laser_callback)
    laser_pub = rospy.Publisher('scan', LaserScan, queue_size=1)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        rospy.spin()