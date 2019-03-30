#!/usr/bin/env python
"""
This is for human detection on Pepper
"""

import rospy
from darknet_ros_msgs.msg import *
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from robot_project.msg import ObjPoseArray, ObjPose
import tf
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


class VideoFrames:
    """
    Reference : ros-video-recorder
    https://github.com/ildoonet/ros-video-recorder/blob/master/scripts/recorder.py
    """

    def __init__(self, image_topic):
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback_image, queue_size=1)
        self.bridge = CvBridge()
        self.frames = []

    def callback_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)

        except CvBridgeError as e:
            rospy.logerr('Converting Image Error. ' + str(e))
            return

        self.frames.append((data.header.stamp, cv_image))

    def get_latest(self, at_time, remove_older=True):
        fs = [x for x in self.frames if x[0] <= at_time]
        if len(fs) == 0:
            return None

        f = fs[-1]
        if remove_older:
            self.frames = self.frames[len(fs) - 1:]

        return f[1]


def camera2pose(depth, pixel_x, pixel_y, listener, ref_frame, target_frame):

    # x, y, z direction are relative to robot frame (unit: meters)
    obj_depth_frame = PoseStamped()
    obj_depth_frame.header.frame_id = ref_frame
    obj_depth_frame.pose.position.x = np.round(depth / 1000, 4)
    obj_depth_frame.pose.position.y = np.round(((pixel_x - cx) * depth / fx) / 1000, 4)
    obj_depth_frame.pose.position.z = np.round(((cy - pixel_y) * depth / fy) / 1000, 4)
    obj_depth_frame.pose.orientation.w = 1.0  # Neural orientation

    if target_frame != ref_frame:
        obj_target_frame = listener.transformPose(target_frame, obj_depth_frame)  # convert into /map frame

    else:
        obj_target_frame = obj_depth_frame

    return obj_target_frame.pose.position.x, obj_target_frame.pose.position.y


def draw_map(pose_x, pose_y, class_name, temp_map, robot_pose_offset):
    pose_string = '(' + str(pose_x) + ', ' + str(pose_y) + ')'  # pose_x, pose_y unit:m
    pixel_per_length = 1  # unit:1 cm = 1 pixels
    temp_x = temp_map.shape[1] // 2 + int(pose_y * 100) * pixel_per_length
    temp_y = (temp_map.shape[0] - robot_pose_offset) - int(pose_x * 100) * pixel_per_length

    cv2.circle(temp_map, (temp_x, temp_y), 3, (255, 0, 0), 2)
    cv2.putText(temp_map, class_name, (temp_x + 5, temp_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
    cv2.putText(temp_map, pose_string, (temp_x + 5, temp_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

    return


def create_ros_marker(input_obj_array, marker_scale=0.4, red=1.0, green=1.0, blue=0.0, alpha=1.0):  # input point list

    pos = Marker()  # type: Marker
    pos.header.frame_id = 'map'
    pos.header.stamp = rospy.Time.now()
    pos.ns = 'obj_pos'
    pos.type = 8  # point
    pos.action = 0  # Add

    pos.pose.position.x = 0.0
    pos.pose.position.y = 0.0

    pos.pose.orientation.x = 0.0
    pos.pose.orientation.y = 0.0
    pos.pose.orientation.z = 0.0
    pos.pose.orientation.w = 1.0

    pos.scale.x = marker_scale
    pos.scale.y = marker_scale
    pos.scale.z = marker_scale

    for ele in input_obj_array:
        print 'x=', ele.x, ' y=', ele.y
        p = Point()
        p.x = ele.x
        p.y = ele.y
        p.z = 0.05

        c = ColorRGBA()
        c.r = red
        c.g = green
        c.b = blue
        c.a = alpha

        pos.points.append(p)
        pos.colors.append(c)

    return pos


def bd_callback(data):
    global obj_pose_array, obj_buffer

    # get image with pose time
    t = data.header.stamp
    depth_img = df.get_latest(t, remove_older=True)

    if depth_img is None:
        rospy.logwarn('No received depth images.')
        return

    temp_map_width = 512
    robot_pose_offset = 10  # pixel, for draw_map method.
    temp_map = np.ones((temp_map_width, temp_map_width, 3), np.uint8) * 255  # show object location relative to robot

    # Draw a robot
    cv2.circle(temp_map, (temp_map_width // 2, temp_map_width - robot_pose_offset), 3, (0, 0, 255), 2)
    cv2.putText(temp_map, 'robot', (temp_map_width // 2 + 5, temp_map_width - robot_pose_offset),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    obj_list = []
    glob_obj_list = []
    rospy.set_param("human_detected", False)
    person_in_frame = 0  # number of person frame
    for obj in data.bounding_boxes:

        if obj.Class == 'person':
            bound_depth_img = np.copy(depth_img[obj.ymin:obj.ymax, obj.xmin:obj.xmax])
            bound_depth_img[bound_depth_img > 5000] = 5000  # max range = 5m

            # slide_width = np.min([bound_depth_img.shape[0], bound_depth_img.shape[1]]) // 4  # for object
            slide_width = 30  # for human detection

            sxmin = bound_depth_img.shape[1] // 2 - slide_width // 2
            sxmax = bound_depth_img.shape[1] // 2 + slide_width // 2
            symin = bound_depth_img.shape[0] // 2 - slide_width // 2
            symax = bound_depth_img.shape[0] // 2 + slide_width // 2

            search_box = bound_depth_img[symin:symax, sxmin:sxmax]
            depth_val = np.mean(search_box)

            obj_pixel_x = obj.xmin + bound_depth_img.shape[1] // 2
            obj_pixel_y = obj.ymin + bound_depth_img.shape[0] // 2

            # Draw on depth image
            depth_img = cv2.circle(depth_img, (obj.xmin + bound_depth_img.shape[1] // 2,
                                               obj.ymin + bound_depth_img.shape[0] // 2), 3, 100, 2)
            depth_img = cv2.rectangle(depth_img, (obj.xmin + sxmin, obj.ymin + symin),
                                      (obj.xmin + sxmax, obj.ymin + symax), 200, 2)

            depth_img = cv2.rectangle(depth_img, (obj.xmin, obj.ymin), (obj.xmax, obj.ymax), 255, 2)

            # Convert camera data to pose relative to robot depth camera
            obj_x, obj_y = camera2pose(depth_val, obj_pixel_x, obj_pixel_y,
                                       tf_listener, "CameraTop_frame", "CameraTop_frame")

            temp_obj = ObjPose()  # type: ObjPose
            temp_obj.Class = obj.Class
            temp_obj.x = obj_x
            temp_obj.y = obj_y

            if obj_buffer.Class == '' and obj_buffer.x == 0.0 and obj_buffer.y == 0.0:  # if buffer is empty
                obj_buffer = temp_obj

            else:
                perturbation = np.round(
                    np.sqrt((obj_buffer.x - temp_obj.x) ** 2.0 + (obj_buffer.y - temp_obj.y) ** 2.0), 2)

                if perturbation < (0.22 * temp_obj.x + 0.01 * temp_obj.y):  # unit: meter
                    obj_buffer = temp_obj

            # draw_map(temp_obj.x, temp_obj.y, temp_obj.Class, temp_map, robot_pose_offset)
            draw_map(obj_buffer.x, obj_buffer.y, obj_buffer.Class, temp_map, robot_pose_offset)
            obj_list.append(obj_buffer)  # record all the object, not for single human detection

            # Convert points to map frame
            glob_obj_pose = ObjPose()
            glob_obj_pose.Class = obj_buffer.Class
            glob_obj_pose.x, glob_obj_pose.y = camera2pose(depth_val, obj_pixel_x, obj_pixel_y,
                                                           tf_listener, "CameraTop_frame", "map")
            glob_obj_list.append(glob_obj_pose)

            # set human_detected is True
            rospy.set_param("human_detected", True)
            person_in_frame += 1

    if person_in_frame == 0:  # no human in current frame
        rospy.set_param("human_detected", False)
        obj_buffer.Class = ''
        obj_buffer.x = 0.0
        obj_buffer.y = 0.0
        obj_list = []  # reset everything

    # Publish obj_list
    obj_pose_array = obj_list
    obj_pose_pub.publish(obj_pose_array)

    # draw human marker on rviz
    obj_marker = create_ros_marker(input_obj_array=glob_obj_list)
    obj_mark_pub.publish(obj_marker)

    depth_box_pub.publish(cv_bridge.cv2_to_imgmsg(depth_img))
    temp_map_pub.publish(cv_bridge.cv2_to_imgmsg(temp_map, 'bgr8'))

    return


def get_cam_info(data):  # camera_info
    global fx, fy, cx, cy, cam_width, cam_height

    cam_width, cam_height = data.width, data.height
    fx, cx, fy, cy = data.K[0], data.K[2], data.K[4], data.K[5]
    rospy.loginfo('get camera info done!')
    cam_info_sub.unregister()  # subscribe only once

    return


if __name__ == '__main__':
    rospy.init_node('get_human', anonymous=True)
    rospy.loginfo('get_human initialization')

    # initialization (global variables)
    tf_listener = tf.TransformListener()
    depth_topic = '/naoqi_driver_node/camera/depth/image_raw'  # for pepper
    camera_info_topic = '/naoqi_driver_node/camera/front/camera_info'  # for pepper

    if not rospy.has_param("human_detected"):
        rospy.set_param("human_detected", False)

    # Camera info subscribe once
    fx, fy, cx, cy, cam_width, cam_height = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    cam_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, get_cam_info)  # only subscribe once

    obj_pose_array = ObjPoseArray()
    cv_bridge = CvBridge()
    df = VideoFrames(depth_topic)  # get depth image

    obj_buffer = ObjPose()  # for single object(human)

    rospy.loginfo('Wait for depth_topic')
    rospy.wait_for_message(depth_topic, Image, timeout=30)

    # Publisher
    depth_box_pub = rospy.Publisher("/thesis/depth_box", Image, queue_size=1)
    temp_map_pub = rospy.Publisher("/thesis/robot_human_location", Image, queue_size=1)
    obj_pose_pub = rospy.Publisher("/thesis/human_location", ObjPoseArray, queue_size=10)
    obj_mark_pub = rospy.Publisher("/thesis/human_marker", Marker, queue_size=1)

    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, bd_callback, queue_size=1)

    rate = rospy.Rate(10)  # unit: hz

    rospy.loginfo('Start finding human!')

    while not rospy.is_shutdown():
        try:
            rate.sleep()

        except rospy.ROSInterruptException:
            rospy.loginfo('Shut down get_human ...')
