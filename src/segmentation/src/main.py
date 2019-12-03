#!/usr/bin/env python
"""Skeleton code for Lab 6
Course: EECS C106A, Fall 2019
Author: Amay Saxena

This file implements a ROS node that subscribes to topics for RGB images,
pointclouds, and camera calibration info, and uses the functions you
implemented to publish a segmented pointcloud to the topic /segmented_points.

Once you are confident in your implementation in image_segmentation.py and
pointcloud_segmentation.py, run this file to begin publishing a segmented
pointcloud.
"""

from __future__ import print_function
from collections import deque

import rospy
import message_filters
import ros_numpy
import tf

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
import numpy as np
import cv2

from cv_bridge import CvBridge

from ball_tracking import tracking

# TODO this can actually change should probably get from a topic
depth_scale = 1e-3  # measurements in mm convert to meter

def get_camera_matrix(camera_info_msg):
    # Return the camera intrinsic matrix as a 3x3 numpy array
    # by retreiving information from the CameraInfo ROS message.
    return np.reshape(camera_info_msg.K,(3,3))

def isolate_object_of_interest(image, cam_matrix, depth, trans, rot):
    print('before tracking')
    camera_point = tracking(image, depth, depth_scale, cam_matrix)
    if camera_point is None:
        return None, None
    world_point = np.dot(rot, camera_point) + trans
    x, y, z = world_point 
    print('transformed point', world_point) 
    point_stamped = PointStamped()
    point_stamped.point.x = x
    point_stamped.point.y = y
    point_stamped.point.z = z
    point_stamped.header.stamp = rospy.Time.now()
    point_stamped.header.frame_id = '/ar_marker_13'

    x, y, z = camera_point 
    camera_point_stamped = PointStamped()
    camera_point_stamped.point.x = x
    camera_point_stamped.point.y = y
    camera_point_stamped.point.z = z
    camera_point_stamped.header.stamp = rospy.Time.now()
    camera_point_stamped.header.frame_id = '/camera_aligned_depth_to_color_frame'
    return point_stamped, camera_point_stamped

class PointcloudProcess:
    """
    Wraps the processing of a pointcloud from an input ros topic and publishing
    to another PointCloud2 topic.

    """
    def __init__(self, 
                       image_sub_topic,
                       cam_info_topic,
                       depth_sub_topic,
                       state_estimate_pub_topic,
                       state_estimate_camera_frame_pub_topic):

        self.num_steps = 0

        self.messages = deque([], 5)
        self.pointcloud_frame = None
        # points_sub = message_filters.Subscriber(points_sub_topic, PointCloud2)
        image_sub = message_filters.Subscriber(image_sub_topic, Image)
        caminfo_sub = message_filters.Subscriber(cam_info_topic, CameraInfo)
        depth_sub = message_filters.Subscriber(depth_sub_topic, Image)

        self._bridge = CvBridge()
        self.listener = tf.TransformListener()
        ##TODO: publish the state estimate
        self.state_estimate_pub = rospy.Publisher(state_estimate_pub_topic, PointStamped, queue_size=10)
        self.state_estimate_cam_frame_pub = rospy.Publisher(state_estimate_camera_frame_pub_topic, PointStamped, queue_size=10)
        
        ts = message_filters.ApproximateTimeSynchronizer([ image_sub, caminfo_sub, depth_sub],
                                                          10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, image, info, depth):
        try:
            intrinsic_matrix = get_camera_matrix(info)
            rgb_image = ros_numpy.numpify(image)
            depth_image = ros_numpy.numpify(depth)
        except Exception as e:
            rospy.logerr(e)
            return
        self.num_steps += 1
        self.messages.appendleft(( rgb_image, intrinsic_matrix, depth_image))

    def publish_once_from_queue(self):
        if self.messages:
            image, info, depth = self.messages.pop()
            try:
                trans, rot = self.listener.lookupTransform(
                                                       '/ar_marker_13',
                                                       '/camera_aligned_depth_to_color_frame',
                                                       rospy.Time(0))
                rot = tf.transformations.quaternion_matrix(rot)[:3, :3]
            except (tf.LookupException,
                    tf.ConnectivityException, 
                    tf.ExtrapolationException) as e:
                print(e)
                return
            world_point_msg, camera_point_msg = isolate_object_of_interest(image, info, depth,
                np.array(trans), np.array(rot))
            if world_point_msg is not None:
                self.state_estimate_pub.publish(world_point_msg)
                self.state_estimate_cam_frame_pub.publish(camera_point_msg)
                print("Published state estimate at timestamp:",
                        world_point_msg.header.stamp.secs)

def main():
    print('here1')
    #CAM_INFO_TOPIC = '/camera/color/camera_info'
    CAM_INFO_TOPIC = '/camera/aligned_depth_to_color/camera_info'
    DEPTH_TOPIC = '/camera/aligned_depth_to_color/image_raw'
    RGB_IMAGE_TOPIC = '/camera/color/image_raw'
    # POINTS_TOPIC = '/camera/depth/color/points'
    #DEPTH_TOPIC = '/camera/depth/image_rect_raw'
    STATE_ESTIMATE_PUB_TOPIC = '/state_estimate'
    STATE_ESTIMATE_IN_CAMERA_FRAME_TOPIC = '/state_estimate_camera_frame'

    rospy.init_node('realsense_listener')
    process = PointcloudProcess(RGB_IMAGE_TOPIC,
                                CAM_INFO_TOPIC, DEPTH_TOPIC, 
                                STATE_ESTIMATE_PUB_TOPIC, 
                                STATE_ESTIMATE_IN_CAMERA_FRAME_TOPIC)
    r = rospy.Rate(1000)
    print('here2')


    while not rospy.is_shutdown():
        process.publish_once_from_queue()
        r.sleep()

if __name__ == '__main__':
    main()
