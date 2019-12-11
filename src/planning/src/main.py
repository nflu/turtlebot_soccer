#!/usr/bin/env python
"""
Main file for perception module

run this file by running

rosrun perception main.py

"""

from collections import deque

import rospy

from geometry_msgs.msg import PointStamped
import numpy as np
import threading
from prediction.msg import point_vel
import tf
import argparse
import matplotlib.pyplot as plt

POINT_ON_TURTLEBOT = np.zeros(3)
NOMINAL_MAX_SPEED = 0.3

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
                 points_to_check=100,
                 time_horizon=1.0):
        """
        """



        self.verbosity = verbosity

        # queue of messages will be formed from data from subscribers during
        # callback then published
        self.messages = deque([], max_deque_size)
        self.messages_lock = threading.Lock()


        self.intersection_pub_1 = rospy.Publisher(intersection_topic,
                                                PointStamped,
                                                queue_size=queue_size)

        self.intersection_pub_2 = rospy.Publisher(intersection_topic + '2',
                                                PointStamped,
                                                queue_size=queue_size)

        # listener to get transform between world frame and turtlebot frame
        self.listener = tf.TransformListener()
        self.turtlebot_frame = turtlebot_frame

        self.points_to_check = points_to_check
        self.time_horizon = time_horizon

                # set up subscribers
        self.prediction_topic_sub = rospy.Subscriber(prediction_topic,
                                                point_vel,
                                                self.callback)


    def callback(self, point_v):
        """
        """
        # add to queue of messages
        self.messages.appendleft(point_v)

    def publish_once_from_queue(self):
        """
        publishes a prediction from a message in queue
        :return: None
        """
        if self.messages:
            point_v = self.messages.pop()

            x_b = point_v.point.x
            y_b = point_v.point.y
            z = point_v.point.z

            ball_point = np.array([x_b, y_b])

            x_dot = point_v.linear.x
            y_dot = point_v.linear.y

            ball_velocity = np.array([x_dot, y_dot])

            point = PointStamped()
            point.point = point_v.point
            point.header = point_v.header
            
            
            world_frame = point_v.header.frame_id

            try:
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

            point.header.stamp = rospy.Time.now()
           # self.intersection_pub_1.publish(point)
            
            turtle_point = np.dot(rot, POINT_ON_TURTLEBOT) + trans
            turtle_point = turtle_point[:2]


            speed = np.linalg.norm(ball_velocity)
            distance = np.linalg.norm(ball_point - turtle_point)

            ts = np.linspace(0, self.time_horizon, self.points_to_check)

            best_dist = float('inf')
            best_point = None
            best_turtle_point = None
            for t in ts:
                ball_next_point = ball_point + ball_velocity * t
                v = ball_next_point - turtle_point
                v = v / np.linalg.norm(v) 
                turtle_next_point = turtle_point + v * NOMINAL_MAX_SPEED * t
                if np.linalg.norm(turtle_next_point - ball_next_point) < best_dist:
                    best_point = ball_next_point
                    best_turtle_point = turtle_next_point

            point = make_point_stamped(best_point[0], best_point[1], z, rospy.Time.now(), world_frame)
            point2 = make_point_stamped(best_turtle_point[0], best_turtle_point[1], z, rospy.Time.now(), world_frame)
            self.intersection_pub_1.publish(point)
            self.intersection_pub_2.publish(point)
            #ball_path = ball_point + ball_velocity * ts
            #turtle_path = turtle_point + NOMINAL_MAX_SPEED * ts




def main():

    # set up command line arguments
    parser = argparse.ArgumentParser(description="Planning Module")
    parser.add_argument('--turtlebot_frame', type=str, help='defaults to base_link')
    
    # parse arguments
    args = parser.parse_args()
    turtlebot_frame = args.turtlebot_frame if args.turtlebot_frame else 'base_link'

    # subscribing topics
    prediction_topic = '/predicted_path'

    # publishing topics
    intersection_topic = '/intersection_point'

    rospy.init_node('planning')

    process = PlanningProcess(prediction_topic=prediction_topic,
                              intersection_topic=intersection_topic,
                              turtlebot_frame=turtlebot_frame)
                                
    r = rospy.Rate(1000)

    # run perception
    while not rospy.is_shutdown():
        process.publish_once_from_queue()
        r.sleep()


if __name__ == '__main__':
    main()
