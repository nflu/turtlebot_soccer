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


# Actual speeds (roughly) of the Turtlebot in m/s and rad/s
MAX_SPEED = 0.5
MAX_OMEGA = 1.9

class Controller:
    
    def __init__(self, turtlebot_frame, velocity_topic, current_point_topic, queue_size=10, max_deque_size=5):
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

        self.velocity_sub = rospy.Subscriber(velocity_topic, PointStamped, self.velocity_callback)
        self.curr_pos_sub = rospy.Subscriber(current_point_topic, PointStamped, self.curr_pos_callback)
        self.velocity_messages = deque([], max_deque_size)
        self.curr_pos_messages = deque([], max_deque_size)

        # Create a timer object that will sleep long enough to result in
        # a 10Hz publishing rate
        self.r = rospy.Rate(10)  # 10hz

        self.turtlebot_frame = turtlebot_frame
    
    def velocity_callback(self, point):
        self.velocity_messages.appendleft(point)
    
    def curr_pos_callback(self, point):
        self.curr_pos_messages.appendleft(point)
    
    def publish_once_from_queue(self):
        velocity = self.velocity_messages.pop()
        curr_point = self.curr_pos_messages.pop()
        #time_of_sensor_measurement = curr_point.header.stamp
        predictor_planner = Predictor_Planner(velocity.point.x, curr_point.point.x, velocity.point.y, curr_point.point.y)
        robot_state = [curr_point.point.x, curr_point.point.y]
        plan = predictor_planner.plan_to_intercept(robot_state)
        #time_of_plan = rospy.Time.now()
        time_achieved = plan.planning_time()

        #time_since_plan = rospy.Time.now() - time_of_plan
        #converted_time = time_since_plan/time_achieved 	# Plan is normalized to happen in only 1 sec
        #control_speed = plan.speed(converted_time)/time_achieved
        #control_omega = plan.omega(converted_time)/time_achieved
        control_x_vel = plan.x_dot(0)/time_achieved
        control_y_vel = plan.y_dot(0)/time_achieved
        control_omega = plan.omega(0)/time_achieved

        # Generate a control command to send to the robot
        msg = Twist()
        msg.linear.x = control_x_vel
        msg.linear.y = control_y_vel
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = control_omega

        self.pub.publish(msg)
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
    def __init__(self, m1, b1, m2, b2):
        self.m1 = m1
        self.b1 = b1
        self.m2 = m2
        self.b2 = b2

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

    def plan_to_intercept(self, robot_state):
        # Finds a plan to intercept the ball

        for t in range(100):
            # Where is the ball? That's the goal for the planner
            at_t_x, at_t_y = self.future_location(t) 
            robot_theta = np.pi   ######################## THIS IS A PLACEHOLDER #######################
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
    # Run this program as a new node in the ROS computation graph
    # called /turtlebot_controller.
    rospy.init_node('turtlebot_controller', anonymous=True)

    turtlebot_frame = 'base_link'
    velocity_topic = '/predicted_path'
    current_point_topic = '/avg_state_est'

    try:
        controller = Controller(turtlebot_frame=turtlebot_frame, velocity_topic=velocity_topic, current_point_topic=current_point_topic)
        
        while True:
            controller.publish_once_from_queue()
    except rospy.ROSInterruptException:
        pass
