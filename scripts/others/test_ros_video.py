#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
This is for loading videos and publish into ros image topic
"""
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import os


class ImageConverter:
    def __init__(self):
        self.video_name = '/home/robot/NTU_RGBD/nturgb+d_rgb_3/S003C003P007R001A001_rgb.avi'
        self.save_dir = '/home/robot/temp_video2img/'
        self.image_pub = rospy.Publisher('/thesis/img_stitching', Image, queue_size=10)
        self.bridge = CvBridge()
        self.now = rospy.get_rostime()

    def to_imgmsg(self, cv_image):
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            ros_image.header.frame_id = 'usb_cam'
            ros_image.header.seq += 1

            self.now = rospy.get_rostime()
            ros_image.header.stamp.secs = self.now.secs
            ros_image.header.stamp.nsecs = self.now.nsecs

            self.image_pub.publish(ros_image)

        except CvBridgeError as e:
            rospy.logerr(e)
            pass
        return

    def video_to_imgmsg(self, video_name=None, save_dir=None, save_flag=False):
        if video_name is not None:
            self.video_name = video_name
        if save_dir is not None:
            self.save_dir = save_dir

        out_dir = self.video_name.split('/')[-1].split('.')[0]

        if out_dir not in os.listdir(self.save_dir):
            os.mkdir(self.save_dir + out_dir)

        cap = cv2.VideoCapture(self.video_name)
        rospy.loginfo('Video start playing ...')
        start_time = time.time()
        i = 0
        while cap.isOpened():
            ret, frame = cap.read()
            if ret:  # successful read
                # Cropping frame at the middle
                (fh, fw, _) = frame.shape
                crop_img = frame[fh//2-(fw//4):fw//2+(fh//4), fw//2-(fw//8):fw//2+(fw//8), :]
                # crop_img = frame[:, fw//2-(fw//8):fw//2+(fw//8), :]

                if save_flag:
                    cv2.imwrite(self.save_dir + out_dir + '/' + str(i) + '.png', crop_img)
                self.to_imgmsg(crop_img)
                rospy.sleep(0.3)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rospy.loginfo('Processing time: {0}'.format(time.time() - start_time))
                    break

                i += 1
            else:  # no more images
                break

        cap.release()
        return

    def bag_to_mp4(self):
        return


if __name__ == '__main__':
    rospy.init_node(os.path.basename(__file__).split('.')[0], log_level=rospy.INFO)
    ic = ImageConverter()
    ic.video_to_imgmsg(save_dir='/home/robot/temp_images/', save_flag=False)
