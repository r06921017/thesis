#!/usr/bin/env python
# -*- coding: UTF-8 -*-

"""
Turn on tf-openpose with Pepper and visualize with ros imgmsg
Input: images from both top and bottom camera from Pepper
"""

import os
import sys

from threading import Lock
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tfpose_ros.msg import Persons, Person, BodyPartElm

from tf_pose.estimator import TfPoseEstimator
from tf_pose.networks import model_wh, get_graph_path


def humans_to_msg(humans):
    persons = Persons()

    for human in humans:
        person = Person()

        for k in human.body_parts:
            body_part = human.body_parts[k]

            body_part_msg = BodyPartElm()
            body_part_msg.part_id = body_part.part_idx
            body_part_msg.x = body_part.x
            body_part_msg.y = body_part.y
            body_part_msg.confidence = body_part.score
            person.body_part.append(body_part_msg)
        persons.persons.append(person)

    return persons


def callback_image(data):
    if rospy.get_param('/thesis/use_openpose', False):

        try:
            cv_image = cv_bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as err:
            rospy.logerr('[tf-pose-estimation] Converting Image Error. ' + str(err))
            return

        acquired = tf_lock.acquire(False)
        if not acquired:
            return

        try:
            humans = pose_estimator.inference(cv_image, resize_to_default=True, upsample_size=resize_out_ratio)

            # draw
            pose_image = TfPoseEstimator.draw_humans(cv_image, humans, imgcopy=False)
            pub_img.publish(cv_bridge.cv2_to_imgmsg(pose_image, 'bgr8'))

        finally:
            tf_lock.release()

        msg = humans_to_msg(humans)
        msg.image_w = data.width
        msg.image_h = data.height
        msg.header = data.header

        rospy.set_param('/thesis/pose_frame', rospy.get_param('/thesis/img_frame', -1))  # for evaluation
        pub_pose.publish(msg)

    return


if __name__ == '__main__':
    rospy.loginfo('Openpose on Pepper')
    rospy.init_node('TfPoseEstimatorROS', anonymous=True, log_level=rospy.INFO)

    # parameters
    image_topic = rospy.get_param('/thesis/camera', '/thesis/img_stitching')
    model = rospy.get_param('~model', 'mobilenet_thin')  # cmu

    resolution = rospy.get_param('~resolution', '320x240')  # w x h
    resize_out_ratio = float(rospy.get_param('~resize_out_ratio', '4.0'))
    tf_lock = Lock()

    if not image_topic:
        rospy.logerr('Parameter \'camera\' is not provided.')
        sys.exit(-1)

    try:
        w, h = model_wh(resolution)
        graph_path = get_graph_path(model)

        rospack = rospkg.RosPack()
        graph_path = os.path.join(rospack.get_path('tfpose_ros'), graph_path)
    except Exception as e:
        rospy.logerr('invalid model: %s, e=%s' % (model, e))
        sys.exit(-1)

    path = graph_path
    pose_estimator = TfPoseEstimator(path, target_size=(w, h))
    cv_bridge = CvBridge()

    rospy.Subscriber(image_topic, Image, callback_image, queue_size=1, buff_size=2**24)
    pub_pose = rospy.Publisher('/thesis/human_pose', Persons, queue_size=1)
    pub_img = rospy.Publisher('/thesis/pose_visualization', Image, queue_size=1)

    rospy.loginfo('pepper_openpose start+')
    rospy.spin()
    rospy.loginfo('finished')
