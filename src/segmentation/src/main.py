#!/usr/bin/env python
"""
Main file for perception module

run this file by running

rosrun segmentation main.py

"""

#TODO rename segmentation to perception
# use this https://answers.ros.org/question/28165/renaming-a-package/?answer=285084#post-id-285084
from collections import deque

import rospy
import message_filters
import ros_numpy
import tf

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import numpy as np

from ball_tracking import tracking
import argparse

# TODO this can actually change should probably get from a topic
depth_scale = 1e-3  # measurements in mm convert to meter


def get_camera_matrix(camera_info_msg):
    # Return the camera intrinsic matrix as a 3x3 numpy array
    # by retreiving information from the CameraInfo ROS message.
    return np.reshape(camera_info_msg.K,(3,3))


def isolate_object_of_interest(image, cam_matrix, depth, trans, rot):
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


class BallTrackingProcess:

    def __init__(self,
                 image_sub_topic,
                 cam_info_topic,
                 depth_sub_topic,
                 ball_position_pub_topic,
                 ball_position_camera_frame_pub_topic):

        # set up subscribers
        image_sub = message_filters.Subscriber(image_sub_topic, Image)
        cam_info_sub = message_filters.Subscriber(cam_info_topic, CameraInfo)
        depth_sub = message_filters.Subscriber(depth_sub_topic, Image)

        # needed to get transform between camera frame and AR tag frame
        self.listener = tf.TransformListener()

        #
        self.ball_position_pub = rospy.Publisher(ball_position_pub_topic,
                                                  PointStamped,
                                                  queue_size=10)
        self.state_estimate_cam_frame_pub = rospy.Publisher(ball_position_camera_frame_pub_topic,
                                                            PointStamped,
                                                            queue_size=10)

        # messages will be formed from data from subscribers during callback
        # then published
        self.messages = deque([], 5)

        ts = message_filters.ApproximateTimeSynchronizer([image_sub,
                                                          cam_info_sub,
                                                          depth_sub],
                                                         10,
                                                         0.1,
                                                         allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, image, info, depth):
        try:
            intrinsic_matrix = get_camera_matrix(info)
            rgb_image = ros_numpy.numpify(image)
            depth_image = ros_numpy.numpify(depth)
        except Exception as e:
            rospy.logerr(e)
            return
        self.messages.appendleft((rgb_image, intrinsic_matrix, depth_image))

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
            world_point_msg, camera_point_msg = isolate_object_of_interest(image,
                                                                           info,
                                                                           depth,
                                                                           np.array(trans),
                                                                           np.array(rot))
            if world_point_msg is not None:
                self.state_estimate_pub.publish(world_point_msg)
                self.state_estimate_cam_frame_pub.publish(camera_point_msg)
                print("Published state estimate at timestamp:",
                        world_point_msg.header.stamp.secs)


def main():

    # set up command line arguments
    parser = argparse.ArgumentParser(description="Perception Module")
    parser.add_argument("-a", "--ar_tag_number", type=str,
                        help="defaults to 13")  # optional
    parser.add_argument('--debug', action='debug')  # optional

    # parse arguments
    args = parser.parse_args()
    ar_tag_number = args.ar_tag_number if args.ar_tag_number else '15'
    debug = args.debug

    # subscription topics
    rgb_topic = '/camera/color/image_raw'
    cam_info_topic = '/camera/aligned_depth_to_color/camera_info'
    depth_topic = '/camera/aligned_depth_to_color/image_raw'

    # publishing topics
    ball_position_world_topic = '/ball_position_world'
    ball_position_camera_topic = '/state_estimate_camera' if debug else None

    # setup ros subs, pubs and connect to realsense
    rospy.init_node('realsense_listener')
    process = BallTrackingProcess(rgb_topic,
                                  cam_info_topic,
                                  depth_topic,
                                  ball_position_world_topic,
                                  ball_position_camera_topic)
    r = rospy.Rate(1000)

    # run perception
    while not rospy.is_shutdown():
        process.publish_once_from_queue()
        r.sleep()


if __name__ == '__main__':
    main()
