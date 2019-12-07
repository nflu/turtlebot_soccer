#!/usr/bin/env python
"""
Main file for perception module

run this file by running

rosrun perception main.py

"""

from collections import deque

import rospy

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
import numpy as np


class PredictionProcess:

    def __init__(self,
                 state_est_topic,
                 prediction_topic,
                 predicted_point_topic,
                 averaged_state_est_topic,
                 verbosity=2,
                 max_deque_size=5,
                 queue_size=10,
                 max_delta_t=1e8,
                 avg_size=12):
        """
        """

        # set up publishing topics
        self.prediction_pub = rospy.Publisher(prediction_topic,
                                                  Path,
                                                  queue_size=queue_size)
        self.predicted_point_pub = rospy.Publisher(predicted_point_topic,
                                                  PointStamped,
                                                    queue_size=queue_size)
        self.average_point_pub = rospy.Publisher(averaged_state_est_topic,
                                                PointStamped,
                                                queue_size=queue_size)
        self.average_point_pub_2 = rospy.Publisher(averaged_state_est_topic+"2",
                                                PointStamped,
                                                queue_size=queue_size)
        self.verbosity = verbosity

        # queue of messages will be formed from data from subscribers during
        # callback then published
        self.messages = deque([], max_deque_size)
        self.points = []
        self.times = []
        self.avg_size = avg_size
        self.max_delta_t = max_delta_t

        # set up subscribers
        state_est_sub = rospy.Subscriber(state_est_topic, PointStamped, self.callback)

    def callback(self, state_estimate):
        """
        """
        # add to queue of messages
        self.messages.appendleft(state_estimate)

    def publish_once_from_queue(self):
        """
        publishes a prediction from a message in queue
        :return: None
        """
        if self.messages:
            # take a message off the queue
            state_estimate = self.messages.pop()
            x = state_estimate.point.x
            y = state_estimate.point.y
            z = state_estimate.point.z
            if len(self.points) == self.avg_size:
                bin_1_point = np.mean(self.points[:self.avg_size//2], axis=0)
                bin_2_point = np.mean(self.points[self.avg_size//2:], axis=0)
                bin_1_time = np.mean(self.times[:self.avg_size//2])
                bin_2_time = np.mean(self.times[self.avg_size//2:])

                delta_t = bin_2_time - bin_1_time
                if delta_t <= self.max_delta_t:
                    x_dot,y_dot = (bin_2_point-bin_1_point)/delta_t

                    #times = np.arange(0, 1.0, delta_t)
                    #path_y_positions = y_dot*times+y
                    #path_x_positions = x_dot*times+x
                    now = rospy.Time.now()

                    averaged_point2 = PointStamped()
                    averaged_point2.point.x = bin_2_point[0]
                    averaged_point2.point.y = bin_2_point[1]
                    averaged_point2.point.z = z
                    averaged_point2.header.stamp = now
                    averaged_point2.header.frame_id = state_estimate.header.frame_id

                    averaged_point = PointStamped()
                    averaged_point.point.x = bin_1_point[0]
                    averaged_point.point.y = bin_1_point[1]
                    averaged_point.point.z = z
                    averaged_point.header.stamp = now
                    averaged_point.header.frame_id = state_estimate.header.frame_id


                    predicted_point = PointStamped()
                    predicted_point.point.x = x_dot*1e9+averaged_point.point.x
                    predicted_point.point.y = y_dot*1e9+averaged_point.point.y
                    predicted_point.point.z = z
                    predicted_point.header.stamp = now
                    predicted_point.header.frame_id = state_estimate.header.frame_id

                    self.average_point_pub_2.publish(averaged_point2)
                    self.predicted_point_pub.publish(predicted_point)
                    self.average_point_pub.publish(averaged_point)
                    # self.prediction_pub.publish(path)
                    if self.verbosity >= 1:
                        print("Published prediction at timestamp:",
                              predicted_point.header.stamp.nsecs)
                self.points.pop(0)
                self.points.append(np.array([x,y]))
                self.times.pop(0)
                self.times.append(state_estimate.header.stamp.nsecs)
            else:
                self.points.append(np.array([x,y]))
                self.times.append(state_estimate.header.stamp.nsecs)




def main():

    # subscribing topics
    state_est_topic = '/state_estimate'

    # publishing topics
    prediction_topic = '/predicted_path'
    predicted_point_topic = '/predicted_point'
    averaged_state_est_topic = '/avg_state_est'
    # setup ros subs, pubs and connect to realsense
    rospy.init_node('prediction')
    process = PredictionProcess(state_est_topic=state_est_topic,
                                predicted_point_topic=predicted_point_topic,
                                prediction_topic=prediction_topic,
                                averaged_state_est_topic=averaged_state_est_topic)
    r = rospy.Rate(1000)

    # run perception
    while not rospy.is_shutdown():
        process.publish_once_from_queue()
        r.sleep()


if __name__ == '__main__':
    main()
