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
NOMINAL_MAX_SPEED = 0.5

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
                 queue_size=10):
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

            x_dot = point_v.linear.x
            y_dot = point_v.linear.y

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
            x_t, y_t, _ = turtle_point

            speed = np.linalg.norm([x_dot, y_dot])
            distance = np.linalg.norm([x_b-x_t,y_b-y_t])

            h_1 = x_dot**2 + y_dot**2 - NOMINAL_MAX_SPEED**2
            h_2 = (x_b - x_t) * x_dot + (y_b - y_t) * y_dot
            h_3 = (x_b - x_t)**2 + (y_b - y_t)**2

            # TODO maybe check if sufficiently close
            if h_1 == 0:
                t = -h_3/(2*h_2)
                if t >= 0:
                    point_1 = make_point_stamped(x_b + x_dot*t_1, y_b+y_dot*t, z, now, world_frame)
                    self.intersection_pub_1.publish(point_1)
                    if speed > 0.06:
                        print('speed', speed)
                        print('distance', distance)
                   # print("intersection t", t)
                else:
                    print('no solution')
            else:
                in_sqrt = (h_2/h_1)**2 - (h_3/h_1)
                if in_sqrt < 0:
                    print('no solution')
                else:
                    t_1 = -h_2/h_1 + in_sqrt**0.5
                    t_2 = -h_2/h_1 - in_sqrt**0.5
                  #  print("intersection t1", t_1, "intersection t2", t_2)
                    now = rospy.Time.now()
                    if t_1 >= 0:
                        point_1 = make_point_stamped(x_b + x_dot*t_1, y_b+y_dot*t_1, z, now, world_frame)
                        self.intersection_pub_1.publish(point_1)
                        if speed > 0.06:
                            print('speed', speed)
                            print('distance', distance)
                    if t_2 >= 0:
                        point_2 = make_point_stamped(x_b + x_dot*t_2, y_b+y_dot*t_2, z, now, world_frame)
                        if t_1 < 0:
                            self.intersection_pub_1.publish(point_2)
                            if speed > 0.06:
                                print('speed', speed)
                                print('distance', distance)
                        else:
                            self.intersection_pub_2.publish(point_2)
                    if t_1 < 0 and t_2 < 0:
                        print('no solution')
            


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
