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
import subprocess
import argparse


class ImageConverter:
    def __init__(self):
        self.video_dir = '/home/robot/pepper_data/action_videos/'
        # self.video_dir = '/home/robot/NTU_RGBD/nturgb+d_rgb_4/'
        self.video_name = args.video_name
        self.bag_dir = '/home/robot/pepper_data/actions/'
        self.image_dir = '/home/robot/pepper_data/action_images/'
        self.image_pub = rospy.Publisher('/thesis/img_stitching', Image, queue_size=10)
        self.bridge = CvBridge()
        self.now = rospy.get_rostime()
        self.fps = 4.0

    def pub_imgmsg(self, cv_image):
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

    def play_video(self, video_name=None, image_dir=None, save_flag=False):
        if video_name is not None:
            self.video_name = video_name
        if image_dir is not None:
            self.image_dir = image_dir

        cap = cv2.VideoCapture(self.video_dir+self.video_name)
        rospy.set_param('/thesis/video_name', self.video_name)
        rospy.loginfo('Video {0} start playing ...'.format(self.video_name))
        start_time = time.time()
        i = 0
        while cap.isOpened():
            ret, frame = cap.read()
            if ret:  # successful read
                # Cropping frame at the middle
                (fh, fw, _) = frame.shape

                crop_img = frame
                # crop_img = frame[fh//2-(fw//4):fw//2+(fh//4), fw//2-(fw//6):fw//2+(fw//8), :]
                # crop_img = frame[fh//2-(fw//16):, fw//2-(fw//20):fw//2+(fw//4), :]

                if save_flag:
                    if self.video_name.split('.')[0] not in os.listdir(self.image_dir):
                        os.mkdir(self.image_dir + self.video_name.split('.')[0])

                    img_name = (self.image_dir + self.video_name.split('.')[0] + '/' + '{0:03d}.png').format(i)
                    print img_name
                    cv2.imwrite(img_name, crop_img)
                self.pub_imgmsg(crop_img)
                rospy.sleep(1.0/self.fps)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rospy.loginfo('Processing time: {0}'.format(time.time() - start_time))
                    break

                i += 1
            else:  # no more images
                break

        cap.release()
        rospy.loginfo('Done!')
        return

    def play_videos_in_dir(self, save_flag):
        for video in sorted(os.listdir(self.video_dir)):
            if video.split('.')[0] not in os.listdir(self.image_dir):
                self.play_video(video_name=video, save_flag=save_flag)

    def bag_to_video(self, bag_name=None):
        if bag_name is not None:
            c_str = 'gnome-terminal -x sh -c "roslaunch thesis rosbag2video.launch bag_name:=' + bag_name + '"'
            subprocess.call(c_str, shell=True)
        else:
            for bag_name in sorted(os.listdir(self.bag_dir)):
                if bag_name.endswith('.bag'):
                    bag_name = bag_name.split('.')[0]
                    print 'bag_name: ', bag_name
                    if (bag_name + '.avi') not in os.listdir(self.video_dir):
                        c_str = 'roslaunch thesis rosbag2video.launch bag_name:=' + bag_name
                        subprocess.call(c_str, shell=True)
                    rospy.sleep(0.5)
        return

    def play_img(self):
        rospy.set_param('/thesis/video_name', self.video_name)

        rospy.loginfo('Start playing images of {0}!'.format(self.video_name.split('.')[0]))
        rospy.logdebug('{0}'.format(sorted(os.listdir(self.image_dir+self.video_name.split('.')[0]))))

        for img in sorted(os.listdir(self.image_dir+self.video_name.split('.')[0])):
            print '{0}: {1}'.format(self.video_name.split('.')[0], img.split('.')[0])
            rospy.set_param('/thesis/img_frame', img.split('.')[0])
            rospy.sleep(0.01)
            self.pub_imgmsg(cv2.imread(self.image_dir + self.video_name.split('.')[0] + '/' + img))
            rospy.sleep(1.0 / self.fps)
        rospy.loginfo('Done!')
        return


if __name__ == '__main__':
    # add arg parser
    parser = argparse.ArgumentParser(description='Enter video name and function')
    parser.add_argument('--video_name', type=str, default=None)
    parser.add_argument('--function', type=int, required=True,
                        help='0: bag to video, 1: play video without saving, '
                             '2: play video and save into images, 3: play images')

    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node(os.path.basename(__file__).split('.')[0], log_level=rospy.INFO)
    ic = ImageConverter()

    if args.function == 0:
        ic.bag_to_video()

    elif args.function == 1:
        if args.video_name is not None:
            ic.play_video()

    elif args.function == 2:
        if args.video_name is not None:
            ic.play_video(save_flag=True)

    elif args.function == 3:
        ic.play_videos_in_dir(save_flag=True)

    elif args.function == 4:
        ic.play_img()


