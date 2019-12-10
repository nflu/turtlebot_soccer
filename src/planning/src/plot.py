#!/usr/bin/env python
"""
Main file for perception module

run this file by running

rosrun perception main.py

"""

import rospy

from geometry_msgs.msg import PointStamped
import numpy as np
import threading
from prediction.msg import point_vel
import matplotlib.pyplot as plt
import message_filters
import tf


def extract_point(point_stamped):
    return np.array([point_stamped.point.x, point_stamped.point.y])

class PlottingProcess:

    def __init__(self,
                 state_estimate_topic,
                 prediction_topic,
                 intersection_topic,
                 average_state_est_topic,
                 verbosity=2,
                 max_deque_size=20,
                 queue_size=10,
                 slop=0.2):
        """
        """
        self.verbosity = verbosity

        # queue of messages will be formed from data from subscribers during
        # callback then published
        self.preds = []
        self.states = []
        self.intersects = []
        self.avgs = []
        self.messages_lock = threading.Lock()

        # set up subscribers
        self.prediction_sub = message_filters.Subscriber(prediction_topic, PointStamped)
        self.state_estimate_sub = message_filters.Subscriber(state_estimate_topic, PointStamped)
        self.intersection_sub = message_filters.Subscriber(intersection_topic, PointStamped)
        self.average_state_est_sub = message_filters.Subscriber(average_state_est_topic, PointStamped)

        self.listener = tf.TransformListener()
        # uses an adaptive algorithm to match messages based on their timestamp
        ts = message_filters.ApproximateTimeSynchronizer([self.prediction_sub,
                                                          self.state_estimate_sub,
                                                          self.intersection_sub,
                                                          self.average_state_est_sub],
                                                         queue_size=queue_size,
                                                         slop=slop,
                                                         allow_headerless=True)

        ts.registerCallback(self.callback)


    def callback(self, pred_point, state_est_point, intersect_point, avg_point):
        """
        """
        # add to queue of messages
        self.messages_lock.acquire()
        self.preds.append(extract_point(pred_point))
        self.states.append(extract_point(state_est_point))
        self.intersects.append(extract_point(intersect_point))
        self.avgs.append(extract_point(avg_point))
        self.messages_lock.release()

    def publish_once_from_queue(self):
        """
        publishes a prediction from a message in queue
        :return: None
        """
        self.messages_lock.acquire()
        if len(self.preds) >= 3:
            world_frame = '/ar_marker_13'
            turtlebot_frame = '/ar_marker_14'
            try:
                trans, rot = self.listener.lookupTransform(world_frame,
                                                           turtlebot_frame,
                                                           rospy.Time(0))
                # convert quaternion into 3x3 matrix
                rot = tf.transformations.quaternion_matrix(rot)[:3, :3]
            except (tf.LookupException,
                    tf.ConnectivityException, 
                    tf.ExtrapolationException) as e:
                print(e)
                return

            self.preds = np.array(self.preds)
            self.states = np.array(self.states)
            self.intersects = np.array(self.intersects)
            self.avgs = np.array(self.avgs)
            ax = plt.gca()
            plt.scatter(self.preds[:,0], self.preds[:,1], label = 'predicted_point')
            plt.scatter(self.states[:,0], self.states[:,1], label = 'state_estimate')
            plt.scatter(self.intersects[:,0], self.intersects[:,1], label = 'intersect') 
            plt.scatter(self.avgs[:,0],self.avgs[:,1],label = 'average_state_est')
            plt.scatter([trans[0]], [trans[1]], label='turtlebot')
            ax.legend()
            plt.show()

            self.preds = []
            self.states = []
            self.intersects = []
            self.avgs = []
        self.messages_lock.release()


def main():
    rospy.init_node('plotting')
    plotting = PlottingProcess(state_estimate_topic = '/state_estimate',
                              prediction_topic = '/predicted_point',
                              intersection_topic = '/intersection_point',
                              average_state_est_topic = '/avg_state_est')
                                
    r = rospy.Rate(1000)

    # run perception
    while not rospy.is_shutdown():
        plotting.publish_once_from_queue()
        r.sleep()


if __name__ == '__main__':
    main()
