#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Image stitching for YOLO and Openpose on Pepper.
"""

import sys
import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def callback_image(data):

    bottom_msg = rospy.wait_for_message('/naoqi_driver_node/camera/bottom/image_raw', Image)

    try:
        cv_front = cv_bridge.imgmsg_to_cv2(data, "bgr8")
        cv_bottom = cv_bridge.imgmsg_to_cv2(bottom_msg, "bgr8")
        cv_image = np.append(cv_front, cv_bottom, axis=0)
        pub_img.publish(cv_bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    except CvBridgeError as err:
        rospy.logerr('[tf-pose-estimation] Converting Image Error. ' + str(err))

    return


if __name__ == '__main__':
    rospy.loginfo('Image Stitching for Pepper')
    rospy.init_node('img_stitching', anonymous=False, log_level=rospy.INFO)

    # parameters
    image_topic = rospy.get_param('~camera', '/naoqi_driver_node/camera/front/image_raw')

    if not image_topic:
        rospy.logerr('Parameter \'camera\' is not provided.')
        sys.exit(-1)

    cv_bridge = CvBridge()

    rospy.Subscriber(image_topic, Image, callback_image, queue_size=1, buff_size=2**24)
    pub_img = rospy.Publisher('/thesis/img_stitching', Image, queue_size=1)

    rospy.loginfo('start+')
    rospy.spin()
    rospy.loginfo('finished')
