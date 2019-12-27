#!/usr/bin/env python

import rospy
import tf2_ros
import numpy as np
from collections import deque
from geometry_msgs.msg import Twist, PointStamped
import tf
import ros_numpy
import argparse

SEC_TO_NSEC = 1e9


def stamp_to_secs(stamp):
    return np.float128(stamp.secs + stamp.nsecs / SEC_TO_NSEC)


class Controller:

    def __init__(self,
                 turtlebot_frame,
                 sub_topic,
                 turtlebot_color,
                 queue_size=10,
                 max_deque_size=5,
                 verbosity=2):
        """

        :param turtlebot_frame:
        :param sub_topic:
        :param turtlebot_color:
        :param queue_size:
        :param max_deque_size:
        :param turn_mode:
        """

        # set up control publisher. need to specify turtlebot color
        self.pub = rospy.Publisher('/' + turtlebot_color + '/mobile_base/commands/velocity', Twist,
                                   queue_size=queue_size)

        # set up transform listener to get transform between world frame and turtlebot frame
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.turtlebot_frame = turtlebot_frame

        # keep track of the most recent goal so that we can keep publishing control even if planning
        # hasn't sent anything new
        self.most_recent_goal = None
        self.messages = deque([], max_deque_size)

        # Create a timer object that will sleep long enough to result in
        # a 30Hz publishing rate
        self.r = rospy.Rate(30)  # 10hz

        # whether to use arctan for error to make linear velocity more bang bang
        self.use_arctan = True

        # gains. arctan only used for linear and order of gains [linear, angular]
        self.arctan_inner_gain = 12.0
        self.arctan_outer_gain = 0.63
        self.k_p = np.array([0.7, -1.4])

        # where to clip inputs
        self.linear_limit = 1.1
        self.omega_limit = 3

        # gains for integral and derivative term
        self.k_i = np.array([0, 0])
        self.k_d = np.array([0, 0])

        # used to calculate derivative of error
        self.last_error = None
        self.last_time = None

        # used to calculate integral of error
        self.integral_error = 0.0
        self.anti_windup = 1.0

        # how much to print
        self.verbosity = verbosity

        # subscribes to goal point publishing from planner
        self.sub = rospy.Subscriber(sub_topic, PointStamped, self.callback)

    def callback(self, point):
        self.messages.appendleft(point)

    def publish_once_from_queue(self):
        # pop the goal point off the queue if there is one else use the most recent goal point
        self.most_recent_goal = self.messages.pop() if len(self.messages) else self.most_recent_goal
        if self.most_recent_goal is not None:
            try:
                goal_frame = self.most_recent_goal.header.frame_id
                time = stamp_to_secs(self.most_recent_goal.header.stamp)
                point = self.most_recent_goal.point
                trans = self.tfBuffer.lookup_transform(self.turtlebot_frame, goal_frame,
                                                       rospy.Time())
                rot = ros_numpy.numpify(trans.transform.rotation)
                rot = np.array(tf.transformations.quaternion_matrix(rot)[:3, :3])
                # Process trans to get your state error
                # Generate a control command to send to the robot
                msg = Twist()
                point = np.dot(rot, ros_numpy.numpify(point)) + ros_numpy.numpify(trans.transform.translation)

                #                 
                error = np.array([point[1] + 0.3, point[0]])

                # calculate control from error
                msg.linear.x, msg.angular.z = self.control_law(error, time)
                if self.verbosity >= 1:
                    print('linear control:', msg.linear.x, 'angular control:', msg.angular.z)
                self.pub.publish(msg)
                # Use our rate object to sleep until it is time to publish again
                self.r.sleep()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                print(e)
                # any problem send zero control for safety
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            self.pub.publish(msg)

    def control_law(self, error, time):
        if self.verbosity >= 1:
            print('error in y', error[0], 'error in x', error[1])

        # calculate integral error
        self.integral_error = self.anti_windup * self.integral_error + error

        # calculate derivative error
        if self.last_error is not None:
            delta_t = time - self.last_time
            if delta_t != 0:
                derivative_error = (error - self.last_error) / delta_t
            else:
                derivative_error = 0.0
        else:
            derivative_error = np.zeros(2)

        p = self.k_p * error
        i = self.k_i * self.integral_error
        d = self.k_d * derivative_error

        # put linear error through arctan
        if self.use_arctan:
            p[0] = self.k_p[0] * self.arctan_outer_gain * \
                   np.arctan(self.arctan_inner_gain * error[0])

        # update errors
        self.last_error = error
        self.last_time = time

        u = p + i + d

        # clip controls
        u[0] = min(self.linear_limit, max(-self.linear_limit, u[0]))
        u[1] = min(self.omega_limit, max(-self.omega_limit, u[1]))
        return u


if __name__ == '__main__':
    # set up command line arguments
    parser = argparse.ArgumentParser(description="Control Module")
    parser.add_argument('color', type=str, help='color of turtlebot')
    parser.add_argument('turtle_frame', type=str, help='frame of turtlebot')
    parser.add_argument('--goal_topic', type=str, help='defaults to /intersection_point')
    parser.add_argument('--verbosity', type=int, help='defaults to 2')

    # parse arguments
    args = parser.parse_args()
    verbosity = args.verbosity if args.verbosity is not None else 2
    turtlebot_frame = args.turtle_frame
    sub_topic = args.goal_topic if args.goal_topic else '/predicted_point'
    turtlebot_color = args.color

    # start ROS node
    rospy.init_node('turtlebot_controller', anonymous=True)
    
    controller = Controller(turtlebot_frame=turtlebot_frame, sub_topic=sub_topic,
                            turtlebot_color=turtlebot_color, verbosity=verbosity)
    while True:
        # this allows all the set up to be done so when you press enter the robot moves immediately
        raw_input('press enter to run')
        print('\n running!')
        while True:
            controller.publish_once_from_queue()
