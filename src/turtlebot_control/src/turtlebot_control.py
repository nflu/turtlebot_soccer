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

from collections import deque
from geometry_msgs.msg import Twist, PointStamped


# Define the method which contains the main functionality of the node.
class Controller:

    def __init__(self, turtlebot_frame, sub_topic, queue_size=10, max_deque_size=5):
        """
    Controls a turtlebot whose position is denoted by turtlebot_frame,
    to go to a position denoted by target_frame
    Inputs:
    - turtlebot_frame: the tf frame of the AR tag on your turtlebot
    - target_frame: the tf frame of the target AR tag
    """

        # Create a publisher and a tf buffer, which is primed with a tf listener
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=queue_size)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.sub = rospy.Subscriber(sub_topic, PointStamped, self.callback)
        self.most_recent_goal = None
        self.messages = deque([], max_deque_size)

        # Create a timer object that will sleep long enough to result in
        # a 10Hz publishing rate
        self.r = rospy.Rate(10)  # 10hz

        self.k1 = 0.3
        self.k2 = 1

    def callback(self, point):
        self.messages.appendleft(point)

    def publish_once_from_queue(self):
        self.most_recent_goal = self.messages.pop() if len(self.messages) else self.most_recent_goal
        if self.most_recent_goal is not None:
            try:
                goal_frame = self.most_recent_goal.header.frame_id
                point = self.most_recent_goal.point
                trans = self.tfBuffer.lookup_transform(turtlebot_frame, goal_frame, rospy.Time())
                # Process trans to get your state error
                # Generate a control command to send to the robot
                msg = Twist()
                msg.linear.x = self.k1 * (trans.transform.translation.x + point.x)
                msg.angular.z = self.k2 * (trans.transform.translation.y + point.y)

                self.pub.publish(msg)
                # Use our rate object to sleep until it is time to publish again
                self.r.sleep()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                print(e)


# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
    # Check if the node has received a signal to shut down
    # If not, run the talker method

    # Run this program as a new node in the ROS computation graph
    # called /turtlebot_controller.
    rospy.init_node('turtlebot_controller', anonymous=True)

    turtlebot_frame = 'base_link'
    sub_topic = '/state_estimate'

    try:
        controller = Controller(turtlebot_frame, sub_topic)
        while True:
            controller.publish_once_from_queue()
    except rospy.ROSInterruptException:
        pass
