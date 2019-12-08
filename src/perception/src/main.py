#!/usr/bin/env python
"""
Main file for perception module

run this file by running

rosrun perception main.py

"""

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
ball_radius = 0.09398  # in meters amazon says diameter is 7.4 inches
ball_radius = ball_radius * 0.0
SEC_TO_NSEC = 1e9

def get_camera_matrix(camera_info_msg):
    """
    Returns the camera intrinsic matrix as a 3x3 numpy array
    by retrieving information from the CameraInfo ROS message.
    :param camera_info_msg: ROS message of type sensor_msgs/CameraInfo.msg
    :return: K matrix of camera as 3x3 numpy array
    """
    return np.reshape(camera_info_msg.K, (3, 3))


class RealsensePerceptionProcess:

    def __init__(self,
                 world_frame,
                 camera_frame,
                 rgb_topic,
                 cam_info_topic,
                 depth_topic,
                 state_estimate_topic,
                 cam_state_est_topic=None,
                 verbosity=2,
                 max_deque_size=10,
                 queue_size=10,
                 slop=0.1):
        """
        sets up perception process for detecting ball position and turtlebot
        position in world_frame from realsense
        :param rgb_topic: ROS topic of RGB images that will be used to segment
        image and locate pixel location of center of ball and detect AR tags
        :param cam_info_topic: ROS topic of camera info that will be used to
        convert from a pixel with depth location to point in camera frame
        :param depth_topic: ROS topic of depth images that will be used to find
        depth of center of ball. Depth images need to be aligned with RGB images
        make sure to set align_depth:=true when starting the realsense. See
        README for details
        :param state_estimate_topic: topic that state estimate of the ball
        location in world frame will be published to
        :param cam_state_est_topic: topic that state estimate of the ball
        in the camera frame will be published to. This is useful for debugging
        in rviz
        :param world_frame: name of world frame that position of ball will be
        published to
        :param camera_frame: name of camera frame that RGB and depth images
        come from
        :param verbosity: Determines how much to print, display and save. Higher
        verbosity is better for debugging but slower. If set to 0 will only
        print error messages. If set to 1 will also print calculated position of
        ball in camera frame and world frame, pixel location of ball center, and
        the timestamp it was published. If set to 2 will also display RGB and
        depth images with ball detection. If set to 3 will also save RGB, depth,
        blurred images and mask.
        :param max_deque_size: Max size of
        :param queue_size: Maximum size of queue of publishers
        :param slop: delay (in seconds) with which messages can be synchronized
        """

        # set up subscribers
        image_sub = message_filters.Subscriber(rgb_topic, Image)
        cam_info_sub = message_filters.Subscriber(cam_info_topic, CameraInfo)
        depth_sub = message_filters.Subscriber(depth_topic, Image)

        # needed to get transform between camera frame and world frame
        self.listener = tf.TransformListener()
        self.world_frame = world_frame
        self.camera_frame = camera_frame

        # set up publishing topics
        self.state_estimate_pub = rospy.Publisher(state_estimate_topic,
                                                  PointStamped,
                                                  queue_size=queue_size)
        if cam_state_est_topic is None:
            self.debug_pub = None
        else:
            self.debug_pub = rospy.Publisher(cam_state_est_topic,
                                             PointStamped,
                                             queue_size=queue_size)

        self.verbosity = verbosity

        # queue of messages will be formed from data from subscribers during
        # callback then published
        self.messages = deque([], max_deque_size)

        # uses an adaptive algorithm to match messages based on their timestamp
        ts = message_filters.ApproximateTimeSynchronizer([image_sub,
                                                          cam_info_sub,
                                                          depth_sub],
                                                         queue_size=queue_size,
                                                         slop=slop,
                                                         allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, image, info, depth):
        """
        callback used when a message is received from the image, camera info,
        and depth topic. processes message
        :param image: ROS message of type ROS sensor_msgs/Image.msg RGB image
        :param info: ROS message of type sensor_msgs/CameraInfo.msg
        :param depth: ROS message of type ROS sensor_msgs/Image.msg depth image
        :return: None
        """
        try:
            # convert ROS messages into numpy arrays
            intrinsic_matrix = get_camera_matrix(info)
            rgb_image = ros_numpy.numpify(image)
            depth_image = ros_numpy.numpify(depth)
            secs = image.header.stamp.secs
            nsecs = image.header.stamp.nsecs
            timestamp = np.float128(SEC_TO_NSEC * secs + nsecs)
        except Exception as e:
            rospy.logerr(e)
            return
        
        # add to queue of messages
        self.messages.appendleft((rgb_image, intrinsic_matrix, depth_image, timestamp))

    def publish_once_from_queue(self):
        """
        publishes a state estimate from a message in queue
        :return: None
        """
        if self.messages:
            # take a message off the queue
            image, info, depth, image_timestamp = self.messages.pop()

            # get transform between camera frame and world frame
            try:
                trans, rot = self.listener.lookupTransform(self.world_frame,
                                                           self.camera_frame,
                                                           rospy.Time(0))
                # TODO what does this rospy.Time(0) do?
                # convert quaternion into 3x3 matrix
                rot = tf.transformations.quaternion_matrix(rot)[:3, :3]
            except (tf.LookupException,
                    tf.ConnectivityException, 
                    tf.ExtrapolationException) as e:
                print(e)
                return

            world_pt, cam_pt = self.get_ball_location(image, info, depth,
                                                 np.array(trans),
                                                 np.array(rot))
            if world_pt is not None:
                self.state_estimate_pub.publish(world_pt)
                if self.debug_pub is not None:
                    self.debug_pub.publish(cam_pt)
                if self.verbosity >= 1:
                    secs = world_pt.header.stamp.secs
                    nsecs = world_pt.header.stamp.nsecs
                    point_timestamp = np.float128(SEC_TO_NSEC * secs + nsecs)
                    print("Published state estimate at timestamp:",
                          point_timestamp)
                    print("Delay in seconds from image to estimate:",
                            (point_timestamp - image_timestamp) / SEC_TO_NSEC)

    def get_ball_location(self, image, cam_matrix, depth, trans, rot):
        """
        from image, camera info matrix and depth image gets the location of the
        ball in the camera frame. Uses trans and rot to convert point to world
        frame
        :param image: numpy array of RGB image
        :param cam_matrix: numpy array of camera fundamental matrix
        :param depth: numpy array of depth image
        :param trans: numpy array of translation from camera frame to world
        frame
        :param rot: numpy array of rotation from camera frame to world frame
        :return: tuple of (world_point_stamped, camera_point_stamped) each of
        type geometry_msgs/PointStamped.msg
        """
        camera_point = tracking(rgb=image, depth=depth, depth_scale=depth_scale, cam_matrix=cam_matrix,
                                ball_radius=ball_radius, verbosity=self.verbosity)
        if camera_point is None:
            return None, None

        # used to synchronize world and camera frame point
        now = rospy.Time.now()

        x, y, z = camera_point
        camera_point_stamped = PointStamped()
        camera_point_stamped.point.x = x
        camera_point_stamped.point.y = y
        camera_point_stamped.point.z = z
        camera_point_stamped.header.stamp = now
        camera_point_stamped.header.frame_id = self.camera_frame

        world_point = np.dot(rot, camera_point) + trans
        x, y, z = world_point
        world_point_stamped = PointStamped()
        world_point_stamped.point.x = x
        world_point_stamped.point.y = y
        world_point_stamped.point.z = z
        world_point_stamped.header.stamp = now
        world_point_stamped.header.frame_id = self.world_frame

        if self.verbosity >= 1:
            print('point in camera frame:', camera_point)
            print('point in world frame:', world_point)
        return world_point_stamped, camera_point_stamped


def main():

    # set up command line arguments
    parser = argparse.ArgumentParser(description="Perception Module")
    parser.add_argument('--ar_tag_number', type=str, help='defaults to 13')
    parser.add_argument('--verbosity', type=int, help='defaults to 2')

    # parse arguments
    args = parser.parse_args()
    ar_tag_number = args.ar_tag_number if args.ar_tag_number else '13'
    verbosity = args.verbosity if args.ar_tag_number else 2

    # frames
    world_frame = 'ar_marker_' + str(ar_tag_number)
    camera_frame = 'realsense_aligned_depth_to_color_frame'

    # subscription topics
    rgb_topic = '/camera/color/image_raw'
    cam_info_topic = '/camera/aligned_depth_to_color/camera_info'
    depth_topic = '/camera/aligned_depth_to_color/image_raw'

    # publishing topics
    state_est_topic = '/state_estimate'
    cam_state_est_topic = '/state_estimate_camera_frame'

    # setup ros subs, pubs and connect to realsense
    rospy.init_node('realsense_listener')
    process = RealsensePerceptionProcess(world_frame=world_frame,
                                         camera_frame=camera_frame,
                                         rgb_topic=rgb_topic,
                                         cam_info_topic=cam_info_topic,
                                         depth_topic=depth_topic,
                                         state_estimate_topic=state_est_topic,
                                         cam_state_est_topic=cam_state_est_topic,
                                         verbosity=verbosity)
    r = rospy.Rate(1000)

    # run perception
    while not rospy.is_shutdown():
        process.publish_once_from_queue()
        r.sleep()


if __name__ == '__main__':
    main()
