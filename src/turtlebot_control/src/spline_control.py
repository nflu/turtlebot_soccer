#!/usr/bin/env python
# The line above tells Linux that this file is a Python script,
# and that the OS should use the Python interpreter in /usr/bin/env
# to run it. Don't forget to use "chmod +x [filename]" to make
# this script executable.

import rospy
import tf2_ros
from geometry_msgs.msg import Twist, PointStamped
from collections import deque
import tf
import ros_numpy
import numpy as np
import argparse
from prediction.msg import point_vel


# Actual speeds (roughly) of the Turtlebot in m/s and rad/s
MAX_SPEED = 0.5
MAX_OMEGA = 1.9

class Controller:
    
    def __init__(self, turtlebot_frame, world_frame, sub_topic, turtlebot_color, \
        time_horizon=1.0, points_to_check=100, queue_size=10, max_deque_size=5):
        """
        Controls a turtlebot whose position is denoted by turtlebot_frame,
        to go to a position denoted by target_frame
        Inputs:
        - turtlebot_frame: the tf frame of the AR tag on your turtlebot
        - target_frame: the tf frame of the target AR tag
        """

        # Create a publisher and a tf buffer, which is primed with a tf listener
        self.pub = rospy.Publisher('/' + turtlebot_color + '/mobile_base/commands/velocity', Twist, queue_size=queue_size)
        self.prediction_topic_sub = rospy.Subscriber(sub_topic,
                                                point_vel,
                                                self.callback)

        self.messages = deque([], max_deque_size)

        self.time_horizon = time_horizon
        self.points_to_check = points_to_check

        # Create a timer object that will sleep long enough to result in
        # a 30Hz publishing rate
        self.r = rospy.Rate(30) 

        self.turtlebot_frame = turtlebot_frame
        self.world_frame = world_frame
    
    def callback(self, point):
        self.messages.appendleft(point)
    
    def publish_once_from_queue(self):
        if len(self.messages):
            self.most_recent_meas = self.messages.pop()
            curr_point = self.most_recent_meas.point
            velocity = self.most_recent_meas.linear
            predictor_planner = Predictor_Planner(velocity.x, curr_point.x, velocity.y, curr_point.y, self.turtlebot_frame, self.world_frame)
            robot_state = [curr_point.x, curr_point.y]
            self.plan = predictor_planner.plan_to_intercept(robot_state, self.time_horizon, self.points_to_check)
            time_achieved = self.plan.planning_time()  # Plan is normalized to happen in only 1 sec

            control_x_vel = self.plan.x_dot(0)/time_achieved
            control_y_vel = self.plan.y_dot(0)/time_achieved
            control_omega = self.plan.omega(0)/time_achieved

            # Generate a control command to send to the robot
            msg = Twist()
            msg.linear.x = control_x_vel
            msg.linear.y = control_y_vel
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = control_omega

            self.pub.publish(msg)
            self.time = rospy.get_time()
            # Use our rate object to sleep until it is time to publish again
            self.r.sleep()

        else:
            time_achieved = self.plan.planning_time()   # Plan is normalized to happen in only 1 sec
            converted_time = (self.time - rospy.get_time())/time_achieved   
            control_x_vel = self.plan.x_dot(converted_time)/time_achieved
            control_y_vel = self.plan.y_dot(converted_time)/time_achieved
            control_omega = self.plan.omega(converted_time)/time_achieved

            # Generate a control command to send to the robot
            msg = Twist()
            msg.linear.x = control_x_vel
            msg.linear.y = control_y_vel
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = control_omega

            self.pub.publish(msg)
            self.time = rospy.get_time()
            # Use our rate object to sleep until it is time to publish again
            self.r.sleep()



class Plan:
	# Describes a spline
	# x(t) = a1*t^3 + b1*t^2 + c1*t + d1
	# y(t) = a2*t^3 + b2*t^2 + c2*t + d2

	def __init__(self, a1, b1, c1, d1, a2, b2, c2, d2):
		self.a1 = a1
		self.b1 = b1
		self.c1 = c1
		self.d1 = d1

		self.a2 = a2
		self.b2 = b2
		self.c2 = c2
		self.d2 = d2

	def x(self, t):
		return self.a1 * t**3 + self.b1 * t**2 + self.c1 * t + self.d1

	def y(self, t):
		return self.a2 * t**3 + self.b2 * t**2 + self.c2 * t + self.d2	

	def x_dot(self, t):
		return 3 * self.a1 * t**2 + 2 * self.b1 * t + self.c1

	def y_dot(self, t):
		return 3 * self.a2 * t**2 + 2 * self.b2 * t + self.c2

	def x_dbl_dot(self, t):
		return 6 * self.a1 * t + 2 * self.b1

	def y_dbl_dot(self, t):
		return 6 * self.a2 * t + 2 * self.b2

	def speed(self, t):
		return np.sqrt(self.x_dot(t)**2 + self.y_dot(t)**2)

	def theta(self, t):
		return np.arctan2(self.y_dot(t), self.x_dot(t))

	def omega(self, t):
		#return (self.theta(t + 0.01) - self.theta(t - 0.01))/0.02
		return (1/(1 + (self.y_dot(t)/self.x_dot(t)) ** 2)) * \
		(self.x_dot(t) * self.y_dbl_dot(t) - self.y_dot(t) * self.x_dbl_dot(t))/(self.x_dot(t) ** 2)

	def max_speed(self):
		times = np.linspace(0, 1, 1000)
		speeds = [self.speed(t) for t in times]
		return max(speeds)

	def max_omega(self):
		times = np.linspace(0, 1, 1000)
		omegas = [self.omega(t) for t in times]
		return max(omegas) 

	def planning_time(self):
		max_speed = self.max_speed()
		max_omega = self.max_omega()
		if (max_speed/MAX_SPEED) > (max_omega/MAX_OMEGA):
			return max_speed/MAX_SPEED
		else:
			return max_omega/MAX_OMEGA


class Predictor_Planner:
    def __init__(self, m1, b1, m2, b2, turtlebot_frame, world_frame):
        self.m1 = m1
        self.b1 = b1
        self.m2 = m2
        self.b2 = b2
        self.turtlebot_frame = turtlebot_frame
        self.world_frame = world_frame
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    def future_location(self, t):
        at_t_x = self.m1 * t + self.b1
        at_t_y = self.m2 * t + self.b2

        return at_t_x, at_t_y
	
    def line(self, x):
        return (self.m2/self.m1) * (x - self.b1) + self.b2

    def planner(self, initial_state, final_state, initial_speed, final_speed):
        """
        This code computes and plot a third-order spline for a Dubins car from a starting state, non-zero speed 
        to a goal state, non-zero speed. 
        """

        x0 = initial_state[0]
        y0 = initial_state[1]
        theta_0 = initial_state[2]
        vx0 = initial_speed * np.cos(theta_0)
        vy0 = initial_speed * np.sin(theta_0)
        xf = final_state[0]
        yf = final_state[1]
        theta_f = final_state[2]
        vxf = final_speed * np.cos(theta_f)
        vyf = final_speed * np.sin(theta_f)

        # Compute coefficients
        # ds are decided by the initial position
        d1 = x0
        d2 = y0

        # cs are dependent on the initial speed
        c1 = vx0
        c2 = vy0

        # Final state and speed puts some constraints on the remaining coefficients
        a1 = 2 * (x0 - xf) + (vx0 + vxf)
        a2 = 2 * (y0 - yf) + (vy0 + vyf)
        b1 = xf - (2*(x0 - xf) + (vx0 + vxf)) - vx0 - x0
        b2 = yf - (2*(y0 - yf) + (vy0 + vyf)) - vy0 - y0

        return Plan(a1, b1, c1, d1, a2, b2, c2, d2)

    def plan_to_intercept(self, robot_state, time_horizon, points_to_check):
        # Finds a plan to intercept the ball

        ts = np.linspace(0, time_horizon, points_to_check)
        for t in ts:
            # Where is the ball? That's the goal for the planner
            at_t_x, at_t_y = self.future_location(t) 

            trans = self.tfBuffer.lookup_transform(self.turtlebot_frame, self.world_frame, rospy.Time())
            translation = ros_numpy.numpify(trans.transform.translation)
            robot_theta = np.arctan2(translation[1]/translation[0])
            #rot = ros_numpy.numpify(trans.transform.rotation)
            #rot = np.array(tf.transformations.quaternion_matrix(rot)[:3, :3])
            
            final_state = [at_t_x, at_t_y, robot_theta]
            initial_speed = 0.2
            final_speed = 0.2

            robot_state.append(robot_theta)
            plan = self.planner(robot_state, final_state, initial_speed, final_speed)
            time_achieved = plan.planning_time()

            # Can the robot get there in time?
            if time_achieved <= t:
                return plan

        print("Could not find a plan!")
        return None




# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
    # set up command line arguments
    parser = argparse.ArgumentParser(description="Control Module")
    parser.add_argument('color', type=str, help='color of turtlebot')
    parser.add_argument('turtle_frame', type=str, help='frame of turtlebot')
    parser.add_argument('world_frame', type=str, help='frame of AR tag on floor')
    parser.add_argument('--goal_topic', type=str, help='defaults to /intersection_point')

    # parse arguments
    args = parser.parse_args()

    # Run this program as a new node in the ROS computation graph
    # called /turtlebot_controller.
    rospy.init_node('spline_turtlebot_controller', anonymous=True)

    turtlebot_frame = args.turtle_frame if args.turtle_frame else 'base_link'
    sub_topic = args.goal_topic if args.goal_topic else '/predicted_point'
    turtlebot_color = args.color 
    world_frame = args.world_frame if args.world_frame else 'ar_marker_13'
    
    controller = Controller(turtlebot_frame=turtlebot_frame, world_frame=world_frame, sub_topic=sub_topic,
                            turtlebot_color=turtlebot_color)
    while True:
        raw_input('press enter to run')
        print('\n running!')
        while True:
            controller.publish_once_from_queue()

    #turtlebot_frame = 'base_link'
    #prediction_topic = '/predicted_path'
