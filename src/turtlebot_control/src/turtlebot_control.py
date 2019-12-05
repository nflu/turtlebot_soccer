#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import sys
import message_filters

from geometry_msgs.msg import Twist, PointStamped


#Define the method which contains the main functionality of the node.
def controller(turtlebot_frame):
  """
  Controls a turtlebot whose position is denoted by turtlebot_frame,
  to go to a position denoted by target_frame
  Inputs:
  - turtlebot_frame: the tf frame of the AR tag on your turtlebot
  - target_frame: the tf frame of the target AR tag
  """

  ################################### YOUR CODE HERE ##############

  #Create a publisher and a tf buffer, which is primed with a tf listener


def callback(point_stamped):

  point = point_stamped.point 

  print(point_stamped)
  print("________")

  k1 = 0.3
  k2 = 0.5
  # Loop until the node is killed with Ctrl-C
  try:
    trans = tfBuffer.lookup_transform(turtlebot_frame, goal_frame, rospy.Time())

    # Process trans to get your state error
    # Generate a control command to send to the robot
    msg = Twist()
    msg.linear.x = k1 * (trans.transform.translation.x - point.x)
    msg.angular.z = k2 * (trans.transform.translation.y - point.y) # Generate this

    #################################### end your code ###############

    pub.publish(msg)

  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    pass
  # Use our rate object to sleep until it is time to publish again

      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  try:
    turtlebot_frame = sys.argv[1]
    goal_frame = sys.argv[2]
    rospy.init_node('turtlebot_controller', anonymous=True)

    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    sub = rospy.Subscriber('/state_estimate', PointStamped, callback)
    rospy.spin()
  except rospy.ROSInterruptException:
    pass