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

import matplotlib.pyplot as plt


# Actual max speeds (roughly) of the Turtlebot in m/s and rad/s
MAX_SPEED = 0.5
MAX_OMEGA = 1.9


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
		#print("Speeds", speeds)
		return max(speeds)

	def max_omega(self):
		times = np.linspace(0, 1, 1000)
		omegas = [self.omega(t) for t in times]
		#print("Omegas", omegas)
		return max(omegas) 

	def thetas(self):
		times = np.linspace(0, 1, 1000)
		thetas = [self.theta(t) for t in times]
		#print("Thetas", thetas)

	def planning_time(self):
		max_speed = self.max_speed()
		max_omega = self.max_omega()
		if (max_speed/MAX_SPEED) > (max_omega/MAX_OMEGA):
			return max_speed/MAX_SPEED
		else:
			return max_omega/MAX_OMEGA



def planner(initial_state, final_state, initial_speed, final_speed):
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


def controller(initial_state, final_state):
	r = rospy.Rate(30) 
	pub = rospy.Publisher('/green/mobile_base/commands/velocity', Twist, queue_size=10)

	#initial_state = [-1.1000948451086634, 0.29562982549565975, -0.5144221127803001]
	#final_state = [-0.6823950716893242, -0.0015281564722355867, -0.5144221127803001]
	initial_speed = 0.2
	final_speed = 0.2

	plan = planner(initial_state, final_state, initial_speed, final_speed)
	start_time = rospy.get_time()

	while True:
		now = rospy.get_time()
		time_achieved = plan.planning_time()   # Plan is normalized to happen in only 1 sec
		if now - start_time >= time_achieved:
			break
		
		converted_time = (now - start_time)/time_achieved   
		control_x_vel = plan.x_dot(converted_time)/time_achieved
		control_y_vel = plan.y_dot(converted_time)/time_achieved
		control_omega = plan.omega(converted_time)/time_achieved

		msg = Twist()
		msg.linear.x = control_y_vel
		msg.linear.y = control_x_vel
		msg.linear.z = 0
		msg.angular.x = 0
		msg.angular.y = 0
		msg.angular.z = control_omega

		pub.publish(msg)
		r.sleep()




if __name__ == '__main__':
	# initial_state = [-1.1000948451086634, 0.29562982549565975, -0.5144221127803001]
	# final_state = [-0.6823950716893242, -0.0015281564722355867, -0.5144221127803001]
	# initial_speed = 0.2
	# final_speed = 0.2

	# plan = planner(initial_state, final_state, initial_speed, final_speed)


	# plan.thetas()
	# print("Max speed", plan.max_speed())
	# print("Max omega", plan.max_omega())
	# print("Time achieved", plan.planning_time())


	# plt.figure()
	# xs = [plan.x(t) for t in np.linspace(0, 1, 50)]
	# ys = [plan.y(t) for t in np.linspace(0, 1, 50)]
	# plt.plot(xs, ys, 'r--')
	# plt.show()

	rospy.init_node('spline_turtlebot_controller', anonymous=True)
	tfBuffer = tf2_ros.Buffer()
	tfListener = tf2_ros.TransformListener(tfBuffer)

	trans = tfBuffer.lookup_transform("ar_marker_13", "ar_marker_1", rospy.Time())
	translation = ros_numpy.numpify(trans.transform.translation)
	rot = ros_numpy.numpify(trans.transform.rotation)
	rot = np.array(tf.transformations.quaternion_matrix(rot)[:3, :3])
	robot_point1 = [0, 0, 0]
	robot_point2 = [0, 1, 0]
	world_point1 = np.dot(rot, robot_point1) + translation
	world_point2 = np.dot(rot, robot_point2) + translation
	heading_vector = world_point2 - world_point1
	robot_theta = np.arctan2(heading_vector[1], heading_vector[0])

	robot_state = [translation[0], translation[1], robot_theta]
	final_state = [0, 0, robot_theta]

	controller(robot_state, final_state)



