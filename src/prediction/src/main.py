#!/usr/bin/env python
"""
Main file for prediction module

run this file by running

rosrun prediction main.py

"""

from collections import deque

import rospy

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
import numpy as np
import time
import threading
from prediction.msg import point_vel

SEC_TO_NSEC = 1e9

def make_point_stamped(x, y, z, now, frame_id):
    point = PointStamped()
    point.point.x = x
    point.point.y = y
    point.point.z = z
    point.header.stamp = now
    point.header.frame_id = frame_id
    return point

def stamp_to_secs(stamp):
    return np.float128(stamp.secs + stamp.nsecs / SEC_TO_NSEC)

def stamp_to_nsecs(stamp):
    return np.float128(stamp.secs * SEC_TO_NSEC + stamp.nsecs)

class PredictionProcess:

    def __init__(self,
                 state_est_topic,
                 prediction_topic,
                 predicted_point_topic,
                 averaged_state_est_topic,
                 verbosity=2,
                 max_deque_size=20,
                 queue_size=10,
                 max_delta_t=SEC_TO_NSEC * 1.0/20.0,
                 avg_size=5):
        """
        """

        # set up publishing topics
        self.prediction_pub = rospy.Publisher(prediction_topic,
                                                  point_vel,
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
        self.messages_lock = threading.Lock()
        self.points = []
        self.times = []
        self.avg_size = avg_size
        self.max_delta_t = max_delta_t * avg_size

        # set up subscribers
        state_est_sub = rospy.Subscriber(state_est_topic, PointStamped, self.callback)

    def callback(self, state_estimate):
        """
        """
        # add to queue of messages
        self.messages_lock.acquire()
        self.messages.appendleft(state_estimate)
        self.messages_lock.release()

    def publish_once_from_queue(self):
        """
        publishes a prediction from a message in queue
        :return: None
        """
        self.messages_lock.acquire()
        if self.messages:
            state_estimate = self.messages.pop()
        else:
            state_estimate = None
        self.messages_lock.release()

        if state_estimate:
            x = state_estimate.point.x
            y = state_estimate.point.y
            z = state_estimate.point.z
            if len(self.points) == 2 * self.avg_size:
                bin_1_point = np.mean(self.points[:self.avg_size], axis=0)
                bin_2_point = np.mean(self.points[self.avg_size:], axis=0)
                bin_1_time = np.mean(self.times[:self.avg_size])
                bin_2_time = np.mean(self.times[self.avg_size:])

                delta_t = bin_2_time - bin_1_time
                if delta_t <= self.max_delta_t:
                    x_dot,y_dot = (bin_2_point - bin_1_point) / delta_t
                    now = rospy.Time.now()
                    frame_id = state_estimate.header.frame_id
                    averaged_point1 = make_point_stamped(x=bin_1_point[0], y=bin_1_point[1], z=z, 
                                                         now=now, frame_id=frame_id)
                    averaged_point2 = make_point_stamped(x=bin_2_point[0], y=bin_2_point[1], z=z, 
                                                         now=now, frame_id=frame_id)

                    predicted_point = make_point_stamped(x=x_dot*SEC_TO_NSEC+averaged_point2.point.x, 
                                                         y=y_dot*SEC_TO_NSEC+averaged_point2.point.y, z=z, 
                                                         now=now, frame_id=frame_id)

                    point_v = point_vel()
                    # TODO maybe change to averaged_point_2
                    point_v.point = state_estimate.point
                    point_v.linear.x = x_dot * SEC_TO_NSEC
                    point_v.linear.y = y_dot * SEC_TO_NSEC
                    point_v.linear.z = 0
                    point_v.header = predicted_point.header

                    self.average_point_pub_2.publish(averaged_point2)
                    self.predicted_point_pub.publish(predicted_point)
                    self.average_point_pub.publish(averaged_point1)
                    self.prediction_pub.publish(point_v)
                    
                    if self.verbosity >= 1:
                        secs = predicted_point.header.stamp.secs
                        nsecs = predicted_point.header.stamp.nsecs
                        prediction_timestamp = np.float128(secs * SEC_TO_NSEC + nsecs)

                        print("Published prediction at timestamp:",
                              prediction_timestamp)
                        print("Delay from first state estimate to prediction in seconds:",
                            (prediction_timestamp - self.times[0]) / SEC_TO_NSEC)
                        print("Delay from last state estimate to prediction in seconds:",
                            (prediction_timestamp - self.times[-1]) / SEC_TO_NSEC)
                        print('delta t in seconds:', delta_t / SEC_TO_NSEC)
                elif self.verbosity >=1:
                    print('throw away prediction with delta t:', delta_t / SEC_TO_NSEC, 
                        'max_delta_t is:', self.max_delta_t / SEC_TO_NSEC)
                self.points.pop(0)
                self.points.append(np.array([x,y]))
                self.times.pop(0)
                secs = state_estimate.header.stamp.secs
                nsecs = state_estimate.header.stamp.nsecs
                self.times.append(np.float128(SEC_TO_NSEC * secs + nsecs))

            else:
                if self.verbosity >= 1:
                    print('not enough points yet')
                self.points.append(np.array([x,y]))
                secs = state_estimate.header.stamp.secs
                nsecs = state_estimate.header.stamp.nsecs
                self.times.append(np.float128(SEC_TO_NSEC * secs + nsecs))




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
