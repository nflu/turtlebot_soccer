import numpy as np
import matplotlib.pyplot as plt

MAX_SPEED = 3
MAX_OMEGA = 2


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



class BallPredictor:

	def __init__(self):
		self.history = []
		self.m1 = 0
		self.b1 = 0
		self.m2 = 0
		self.b2 = 0

	def record_observation(self, observation):
		if len(self.history) > 10:
			del self.history[0]
		self.history.append(observation)

	def fit(self):
		# Fits a line to the ball's trajectory
		# Solve x(t) = m1 * t + b1
		# 		y(t) = m2 * t + b2

		if len(self.history) < 2:
			return None

		point1 = self.history[-2]
		point2 = self.history[-1]

		A_x = np.array([[point1[2], 1], [point2[2], 1]])
		b_x = np.array([point1[0], point2[0]])
		[m1, b1] = np.linalg.solve(A_x, b_x)
		
		A_y = np.array([[point1[2], 1], [point2[2], 1]])
		b_y = np.array([point1[1], point2[1]])
		[m2, b2] = np.linalg.solve(A_y, b_y)

		self.m1 = m1
		self.b1 = b1
		self.m2 = m2
		self.b2 = b2

	def future_location(self, t):
		at_t_x = self.m1 * t + self.b1
		at_t_y = self.m2 * t + self.b2

		return at_t_x, at_t_y


class Robot:
	'''
	Dynamics:

	x_dot = vcos(theta)
	y_dot = vsin(theta)
	theta_dot = omega
	v_dot = acceleration

	u_1 = omega
	u_2 = acceleration
	
	'''

	def __init__(self, state):
		self.state = state

	#def move(self, speed, omega):



class PredictionSimulator:

	def __init__(self, robot_init_state):
		self.ball_predictor = BallPredictor()
		self.robot_state = robot_init_state


	# def line(self, x, m1, m2, b1, b2):
	# 	m1 = 
	# 	return (m2/m1) * (x - b1) + b2


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



	def plan_to_intercept(self, robot_init_state):
		# Finds a plan to intercept the ball

		for t in range(50):
			# Where is the ball? That's the goal for the planner
			at_t_x, at_t_y = self.ball_predictor.future_location(t)
			final_state = [at_t_x, at_t_y, robot_init_state[2]]
			initial_speed = 1.5
			final_speed = 1.5

			plan = planner(robot_init_state, final_state, initial_speed, final_speed)
			time_achieved = plan.planning_time()

			# Can the robot get there in time?
			if time_achieved <= t:
				return plan



	def simulate(self, ball_observation):
		self.ball_predictor.record_observation(ball_observation)
		if self.ball_predictor.fit() == None:
			print("Not enough observations of the ball!")
			return
		else:
			plan = self.plan_to_intercept(self.robot_state)






if __name__ == "__main__":
	# two_points = [[0, 0, 1], [4, 6, 6]]
	# ([m1, b1], [m2, b2]) = fit(two_points)
	# x = np.linspace(-5, 10, 150)
	# plt.figure()
	# plt.plot(x, line(x, m1, m2, b1, b2))

	# t = 7
	# at_t_x = m1 * t + b1
	# at_t_y = m2 * t + b2

	# plt.scatter([0, 4, at_t_x], [0, 6, at_t_y])
	# plt.show()

	# # Initial and final states
	# initial_state = np.array([0., 0., 0.])
	# final_state = np.array([2., 2., 0.5*np.pi])
	# initial_speed = 10
	# final_speed = 1
	# visualize = True

	# xs, ys, speed, thetas, omegas = plan(initial_state, final_state, initial_speed, final_speed, visualize)

	# plt.show()

	ball_two_points = [[0, 0, 1], [1, 2, 2]]
	robot_init_state = np.array([7., 15., -np.pi/2])

	simulator(ball_two_points, robot_init_state)






# Times at which the Spline is plotted
# time_samples = 100
# times = np.linspace(0., 1., time_samples)
# # Compute the state trajectory at time steps using the above co-efficients
# xs = np.zeros(time_samples)
# ys = np.zeros(time_samples)
# xs_dot = np.zeros(time_samples)
# ys_dot = np.zeros(time_samples)
# xs_dbl_dot = np.zeros(time_samples)
# ys_dbl_dot = np.zeros(time_samples)
# freq = 5

# for i in range(time_samples):
# 	t = times[i]
# 	xs[i] = t*t*(a1*t + b1) + c1*t + d1
# 	ys[i] = t*t*(a2*t + b2) + c2*t + d2
# 	xs_dot[i] = 3*t*t*a1 + 2*t*b1 + c1
# 	ys_dot[i] = 3*t*t*a2 + 2*t*b2 + c2
# 	xs_dbl_dot[i] = 6*t*a1 + 2*b1
# 	ys_dbl_dot[i] = 6*t*a2 + 2*b2

# speed = np.sqrt(xs_dot**2 + ys_dot**2)
# thetas = np.arctan2(ys_dot, xs_dot)

# omegas = np.zeros(time_samples)
# for i in range(len(thetas) - 2):
# 	#derivative = (1/(1 + (ys_dot[i]/xs_dot[i]) ** 2)) * (xs_dot[i] * ys_dbl_dot[i] - ys_dot[i] * xs_dbl_dot[i])/(xs_dot[i] ** 2)
# 	numerical_derivative = (thetas[i + 2] - thetas[i])/(times[i + 2] - times[i])
# 	omegas[i] = numerical_derivative
# omegas[-1] = omegas[-3]
# omegas[-2] = omegas[-3]


# if visualize:
# 	# Let's plot the spline
# 	fig = plt.figure()
# 	ax = fig.add_subplot(111)

# 	ax.plot(xs, ys)
# 	ax.plot(final_state[0], final_state[1], 'ro')
# 	ax.plot(initial_state[0], initial_state[1], 'bo')
# 	# ax.quiver(xs[::freq], ys[::freq], xs_dot[::freq]/speed[::freq], ys_dot[::freq]/speed[::freq], units='width')
# 	ax.quiver(initial_state[0], initial_state[1], np.cos(initial_state[2]), np.sin(initial_state[2]), units='width')
# 	ax.quiver(final_state[0], final_state[1], np.cos(final_state[2]), np.sin(final_state[2]), units='width')

# 	ax.set_xlabel('x')
# 	ax.set_ylabel('y')

# 	ax.set_xlim(-0.5, 2.5)
# 	ax.set_ylim(-0.5, 2.5)

# 	fig = plt.figure()
# 	ax1 = fig.add_subplot(121)
# 	ax1.plot(times, speed, 'r--')
# 	ax1.set_xlabel('time')
# 	ax1.set_ylabel('v')

# 	ax2 = fig.add_subplot(122)
# 	ax2.plot(times, thetas, 'r--')
# 	ax2.set_xlabel('time')
# 	ax2.set_ylabel('theta')

# 	fig = plt.figure()
# 	ax3 = fig.add_subplot(111)
# 	ax3.plot(times, omegas, 'r--')
# 	ax3.set_xlabel('time')
# 	ax3.set_ylabel('omega')

