#!/usr/bin/env python
# The line above tells Linux that this file is a Python script,
# and that the OS should use the Python interpreter in /usr/bin/env
# to run it. Don't forget to use "chmod +x [filename]" to make
# this script executable.

# Import the rospy package. For an import to work, it must be specified
# in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import sys
import numpy as np
from collections import deque
from geometry_msgs.msg import Twist, PointStamped
import tf
import ros_numpy
import argparse
from matplotlib import pyplot as plt

# Define the method which contains the main functionality of the node.
class Controller:

    def __init__(self,
                 turtlebot_frame,
                 sub_topic,
                 turtlebot_color,
                 queue_size=10,
                 max_deque_size=5,
                 turn_mode=False):
        """
        :param turtlebot_frame:
        :param sub_topic:
        :param turtlebot_color:
        :param use_arctan:
        :param queue_size:
        :param max_deque_size:
        :param omega_limit:
        :param linear_limit:
        """

        # Create a publisher and a tf buffer, which is primed with a tf listener
        self.pub = rospy.Publisher('/' + turtlebot_color + '/mobile_base/commands/velocity', Twist, queue_size=queue_size)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.most_recent_goal = None
        self.messages = deque([], max_deque_size)

        # Create a timer object that will sleep long enough to result in
        # a 10Hz publishing rate
        self.r = rospy.Rate(30)  # 10hz

        # control law
        self.turn_mode = turn_mode

        if self.turn_mode:
            self.use_arctan = True
            self.arctan_inner_gain = 10.0
            self.arctan_outer_gain = 0.63
        else:
            self.use_arctan = True
            self.arctan_inner_gain = 10.0
            self.arctan_outer_gain = 0.63

        if self.turn_mode:
            self.k_p = np.array([0.7, -1.1])
        else:
            self.k_p = np.array([0.7, -0.6])

        if self.turn_mode:
            self.omega_limit = 0.76
            self.linear_limit = 1.1
        else:
            self.omega_limit = float('inf')
            self.linear_limit = float('inf')

        self.k_i = np.array([0, 0])

        self.k_d = np.array([0, 0])

        self.last_error = None
        self.last_time = None

        self.integral_error = 0.0
        self.anti_windup = 1.0

        self.turtlebot_frame = turtlebot_frame

        self.sub = rospy.Subscriber(sub_topic, PointStamped, self.callback)

        self.last_msg = None

    def callback(self, point):
        self.messages.appendleft(point)

    def publish_once_from_queue(self):
        self.most_recent_goal = self.messages.pop() if len(self.messages) else self.most_recent_goal
        if self.most_recent_goal is not None:
            try:
                goal_frame = self.most_recent_goal.header.frame_id
                time = np.float128(self.most_recent_goal.header.stamp.secs +
                                   1e-9 * self.most_recent_goal.header.stamp.nsecs)

                point = self.most_recent_goal.point
                trans = self.tfBuffer.lookup_transform(self.turtlebot_frame, goal_frame, rospy.Time())
                rot = ros_numpy.numpify(trans.transform.rotation)
                rot = np.array(tf.transformations.quaternion_matrix(rot)[:3, :3])
                # Process trans to get your state error
                # Generate a control command to send to the robot
                msg = Twist()
                point = np.dot(rot, ros_numpy.numpify(point)) + ros_numpy.numpify(trans.transform.translation)

          
                msg.linear.x, msg.angular.z = self.control_law(np.array([point[1], point[0]]),
                                                                   time)
                print('linear control:', msg.linear.x, 'angular control:', msg.angular.z)
                self.last_msg = msg
                self.pub.publish(msg)
                # Use our rate object to sleep until it is time to publish again
                self.r.sleep()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                print(e)
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            self.pub.publish(msg)

    def control_law(self, error, time):

        self.integral_error = self.anti_windup * self.integral_error + error
        print('error in y', error[0], 'error in x', error[1])
        p = self.k_p * error
        i = self.k_i * self.integral_error

        if self.last_error is not None:
            delta_t = time - self.last_time
            if delta_t != 0:
                d = self.k_d * (error - self.last_error) / delta_t
            else:
                d = 0.0
        else:
            d = np.zeros(2)

        if self.use_arctan:
            p[0] = self.k_p[0] * self.arctan_outer_gain * np.arctan(self.arctan_inner_gain * error[0])

        # update errors
        self.last_error = error
        self.last_time = time

        u = p + i + d
        u[0] = min(self.linear_limit, max(-self.linear_limit, u[0]))
        u[1] = min(self.omega_limit, max(-self.omega_limit, u[1]))
        return u



# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
    # Check if the node has received a signal to shut down
    # If not, run the talker method

    # set up command line arguments
    parser = argparse.ArgumentParser(description="Control Module")
    parser.add_argument('color', type=str, help='color of turtlebot')
    parser.add_argument('turtle_frame', type=str, help='frame of turtlebot')
    parser.add_argument('--goal_topic', type=str, help='defaults to /intersection_point')

    # parse arguments
    args = parser.parse_args()

    # Run this program as a new node in the ROS computation graph
    # called /turtlebot_controller.
    rospy.init_node('turtlebot_controller', anonymous=True)

    turtlebot_frame = args.turtle_frame
    sub_topic = args.goal_topic if args.goal_topic else '/predicted_point'
    turtlebot_color = args.color 
    
    controller = Controller(turtlebot_frame=turtlebot_frame, sub_topic=sub_topic,
                            turtlebot_color=turtlebot_color)
    while True:
        raw_input('press enter to run')
        print('\n running!')
        while True:
            controller.publish_once_from_queue()
