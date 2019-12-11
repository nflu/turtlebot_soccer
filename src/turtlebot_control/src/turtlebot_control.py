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


# Define the method which contains the main functionality of the node.
class Controller:

    def __init__(self, turtlebot_frame, sub_topic, use_arctan=False, queue_size=10, max_deque_size=5):
        """
    Controls a turtlebot whose position is denoted by turtlebot_frame,
    to go to a position denoted by target_frame
    Inputs:
    - turtlebot_frame: the tf frame of the AR tag on your turtlebot
    - target_frame: the tf frame of the target AR tag
    """

        # Create a publisher and a tf buffer, which is primed with a tf listener
        self.pub = rospy.Publisher('/yellow/mobile_base/commands/velocity', Twist, queue_size=queue_size)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.most_recent_goal = None
        self.messages = deque([], max_deque_size)

        # Create a timer object that will sleep long enough to result in
        # a 10Hz publishing rate
        self.r = rospy.Rate(10)  # 10hz

        # control law
        self.use_arctan = use_arctan
        self.arctan_inner_gain = 8.0
        self.arctan_outer_gain = 0.55

        self.k_p = np.array([0.3, -1.0])

        self.k_i = np.array([0, 0])

        self.k_d = np.array([0, 0])

        self.last_error = None
        self.last_time = None

        self.integral_error = 0.0
        self.anti_windup = 1.0

        self.turtlebot_frame = turtlebot_frame
        print('sub topic:', sub_topic)
        self.sub = rospy.Subscriber(sub_topic, PointStamped, self.callback)


    def callback(self, point):
        print('in callback')
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
                rot =  np.array(tf.transformations.quaternion_matrix(rot)[:3, :3])
                # Process trans to get your state error
                # Generate a control command to send to the robot
                msg = Twist()
                point = np.dot(rot, ros_numpy.numpify(point)) + ros_numpy.numpify(trans.transform.translation)

          
                msg.linear.x, msg.angular.z = self.control_law(np.array([point[1], point[0]]),
                                                                   time)

                self.pub.publish(msg)
                # Use our rate object to sleep until it is time to publish again
                self.r.sleep()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                print(e)

    def control_law(self, error, time):

        self.integral_error = self.anti_windup * self.integral_error + error

        p = self.k_p * error
        i = self.k_i * self.integral_error

        if self.last_error is not None:
            delta_t = time - self.last_time
            d = self.k_d * (error - self.last_error) / delta_t
        else:
            d = np.zeros(2)

        if self.use_arctan:
            p[0] = self.arctan_outer_gain * np.arctan(self.arctan_inner_gain * error[0])

        # update errors
        self.last_error = error
        self.last_time = time

        return p + i + d



# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
    # Check if the node has received a signal to shut down
    # If not, run the talker method

    # set up command line arguments
    parser = argparse.ArgumentParser(description="Control Module")
    parser.add_argument('--goal_topic', type=str, help='defaults to /intersection_point')
    parser.add_argument('--use_arctan', type=int, help='defaults to False')

    # parse arguments
    args = parser.parse_args()

    # Run this program as a new node in the ROS computation graph
    # called /turtlebot_controller.
    rospy.init_node('turtlebot_controller', anonymous=True)

    turtlebot_frame = 'base_link'
    sub_topic = args.goal_topic if args.goal_topic else '/predicted_point'
    use_arctan = args.use_arctan if args.use_arctan else False

    try:
        controller = Controller(turtlebot_frame=turtlebot_frame, sub_topic=sub_topic,
                                use_arctan=use_arctan)
        while True:
            controller.publish_once_from_queue()
    except rospy.ROSInterruptException:
        pass
