#!/usr/bin/env python
"""
Main file for planning module

run this file by running

rosrun planning main.py

"""

from collections import deque
import rospy
from geometry_msgs.msg import PointStamped
import numpy as np
import threading
from prediction.msg import point_vel
import tf
import argparse

POINT_ON_TURTLEBOT = np.zeros(3)
NOMINAL_SPEEDS = [0.1, 0.2, 0.3, 0.5]


def make_point_stamped(x, y, z, now, frame_id):
    point = PointStamped()
    point.point.x = x
    point.point.y = y
    point.point.z = z
    point.header.stamp = now
    point.header.frame_id = frame_id
    return point


class PlanningProcess:

    def __init__(self,
                 prediction_topic,
                 intersection_topic,
                 turtlebot_frame,
                 verbosity=2,
                 max_deque_size=20,
                 queue_size=10,
                 points_to_check=50,
                 time_horizon=1.0,
                 epsilon=0.1,
                 smoothed_size=5):

        self.intersection_pub = rospy.Publisher(intersection_topic,
                                                PointStamped,
                                                queue_size=queue_size)

        # listener to get transform between world frame and turtlebot frame
        self.listener = tf.TransformListener()
        self.turtlebot_frame = turtlebot_frame

        # how much to print out
        self.verbosity = verbosity

        # queue of messages will be formed from data from subscribers during
        # callback then published
        self.messages = deque([], max_deque_size)

        # margin of error between robot point and ball point
        self.epsilon = epsilon
        # how many times to check before the time horizon
        self.points_to_check = points_to_check
        # how far to look ahead for an interception point
        self.time_horizon = time_horizon
        # over how many previous points to smooth the goal point
        self.smoothed_size = smoothed_size
        self.previous_goals = []

        # set up subscribers
        self.prediction_topic_sub = rospy.Subscriber(prediction_topic,
                                                point_vel,
                                                self.callback)

    def callback(self, point_v):
        # add to queue of messages
        self.messages.appendleft(point_v)

    def publish_once_from_queue(self):
        """
        publishes a planning point from a message in queue
        :return: None
        """
        if self.messages:
            point_v = self.messages.pop()

            # ball position
            x_b = point_v.point.x
            y_b = point_v.point.y
            z = point_v.point.z

            ball_point = np.array([x_b, y_b])

            # ball velocity
            x_dot = point_v.linear.x
            y_dot = point_v.linear.y

            ball_velocity = np.array([x_dot, y_dot])

            # assume that frame ball is published in is the world frame
            world_frame = point_v.header.frame_id

            try:
                # get transform between world frame and turtlebot frame
                trans, rot = self.listener.lookupTransform(world_frame,
                                                           self.turtlebot_frame,
                                                           rospy.Time(0))
                # convert quaternion into 3x3 matrix
                rot = tf.transformations.quaternion_matrix(rot)[:3, :3]
            except (tf.LookupException,
                    tf.ConnectivityException, 
                    tf.ExtrapolationException) as e:
                print(e)
                return

            # find location of turtlebot in world frame
            turtle_point = np.dot(rot, POINT_ON_TURTLEBOT) + trans
            turtle_point = turtle_point[:2]

            # array of times to check for interception point
            ts = np.linspace(0, self.time_horizon, self.points_to_check)

            for t in ts:
                ball_next_point = ball_point + ball_velocity * t
                v = ball_next_point - turtle_point
                v = v / np.linalg.norm(v) 
                for turtle_speed in NOMINAL_SPEEDS:
                    turtle_next_point = turtle_point + v * turtle_speed * t
                    dist = np.linalg.norm(turtle_next_point - ball_next_point)
                    if dist < self.epsilon:
                        if self.verbosity >= 1:
                            print('found point with dist:', dist)
                            break

            # smoothing on goal point to make controller's job easier
            if len(self.previous_goals) == self.smoothed_size:
                self.previous_goals.pop(0)
            self.previous_goals.append(ball_next_point)
            goal_point = np.mean(self.previous_goals, axis=0)

            interception_point = make_point_stamped(goal_point[0], goal_point[1], z, rospy.Time.now(),
                                       world_frame)
            self.intersection_pub.publish(interception_point)


def main():

    # set up command line arguments
    parser = argparse.ArgumentParser(description="Planning Module")
    parser.add_argument('--turtlebot_frame', type=str, help='defaults to base_link')
    parser.add_argument('--verbosity', type=int, help='defaults to 2')
    
    # parse arguments
    args = parser.parse_args()
    turtlebot_frame = args.turtlebot_frame if args.turtlebot_frame else 'base_link'
    verbosity = args.verbosity if args.verbosity is not None else 2

    # subscribing topics
    prediction_topic = '/ball_point_vel'  # position and velocity of ball

    # publishing topics
    intersection_topic = '/intersection_point'  # point where ball and robot will intersect

    rospy.init_node('planning')

    process = PlanningProcess(prediction_topic=prediction_topic,
                              intersection_topic=intersection_topic,
                              turtlebot_frame=turtlebot_frame,
                              verbosity=verbosity)
                                
    r = rospy.Rate(1000)

    while not rospy.is_shutdown():
        process.publish_once_from_queue()
        r.sleep()


if __name__ == '__main__':
    main()
