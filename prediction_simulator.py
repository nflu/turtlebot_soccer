import numpy as np
import matplotlib.pyplot as plt
import time

# Actual speeds (roughly) of the Turtlebot in m/s and rad/s
MAX_SPEED = 0.6
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

	def plot(self):
		xs = [self.x(t) for t in np.linspace(0, 1, 50)]
		ys = [self.y(t) for t in np.linspace(0, 1, 50)]
		plt.plot(xs, ys, 'r--')
		plt.show()


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
			return False

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

		return True

	def future_location(self, t):
		at_t_x = self.m1 * t + self.b1
		at_t_y = self.m2 * t + self.b2

		return at_t_x, at_t_y


class Robot:
	'''
	Dynamics:

	x_dot = v * cos(theta)
	y_dot = v * sin(theta)
	theta_dot = omega

	u_1 = omega
	u_2 = v
	
	'''

	def __init__(self, x, y, theta, dt):
		self.x = x
		self.y = y
		self.theta = theta
		self.dt = dt

	def move(self, speed, omega):
		self.x = self.x + (self.dt * speed * np.cos(self.theta))
		self.y = self.y + (self.dt * speed * np.sin(self.theta))
		self.theta = self.theta + (self.dt * omega)



class PredictionSimulator:

	def __init__(self, robot_init_state, dt):
		self.ball_predictor = BallPredictor()
		self.dt = dt
		self.robot = Robot(robot_init_state[0], robot_init_state[1], robot_init_state[2], self.dt)

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

	def plan_to_intercept(self, global_time):
		# Finds a plan to intercept the ball

		for t in range(50):
			# Where is the ball? That's the goal for the planner
			at_t_x, at_t_y = self.ball_predictor.future_location(global_time + t)
			final_state = [at_t_x, at_t_y, self.robot.theta]
			initial_speed = 1.5
			final_speed = 1.5

			robot_state = [self.robot.x, self.robot.y, self.robot.theta]
			plan = self.planner(robot_state, final_state, initial_speed, final_speed)
			# xs = [plan.x(t) for t in np.linspace(0, 1, 50)]
			# ys = [plan.y(t) for t in np.linspace(0, 1, 50)]
			# thetas = [plan.theta(t) for t in np.linspace(0, 1, 50)]
			# omegas = [plan.omega(t) for t in np.linspace(0, 1, 50)]
			# plt.plot(xs, ys, 'r--')
			# plt.pause(0.5)
			time_achieved = plan.planning_time()

			# Can the robot get there in time?
			if time_achieved <= t:
				# print("xs: ", xs)
				# print("ys: ", ys)
				# print("thetas: ", thetas)
				# print("omegas: ", omegas)
				# plt.scatter([0, 1], [1, 2])
				# plt.show()
				return plan

		print("Could not find a plan!")
		return None

	def sensor(self, start, period):
		time = 0
		while True:
			yield [start[0] + 0.3 * time, start[1] + 0.3 * time, time]
			time += period

	def simulate(self):
		global_time = 0
		ball_start = [0, 1]
		plt.plot(ball_start[0], ball_start[1], 'mo')
		plt.plot(self.robot.x, self.robot.y, 'ko')
		plt.pause(2)
		sensor_period = 1
		for ball_observation in self.sensor(ball_start, sensor_period):
			if global_time > 15:
				break
			self.ball_predictor.record_observation(ball_observation)
			if self.ball_predictor.fit():
				time_since_planning = 0
				plan = self.plan_to_intercept(global_time)
				xs = [plan.x(t) for t in np.linspace(0, 1, 50)]
				ys = [plan.y(t) for t in np.linspace(0, 1, 50)]
				plt.plot(xs, ys, 'r--')
				plt.pause(2)
				time_achieved = plan.planning_time()
				for i in range(int(sensor_period/self.dt)):
					timestamp = time_since_planning + self.dt*i
					converted_time = timestamp/time_achieved 	# Plan is normalized to happen in only 1 sec
					control_speed = plan.speed(converted_time)
					control_omega = plan.omega(converted_time)
					self.robot.move(control_speed, control_omega)
					plt.plot(ball_observation[0], ball_observation[1], 'mo')
					plt.plot(self.robot.x, self.robot.y, 'ko')
					plt.pause(2)
			
			global_time += sensor_period
		
		plt.show()



	# def simulate(self, ball_observation):
	# 	self.ball_predictor.record_observation(ball_observation)
	# 	if not self.ball_predictor.fit():
	# 		print("Not enough observations of the ball!")
	# 		return
	# 	else:
	# 		plan = self.plan_to_intercept()
	# 		if plan is not None:
	# 			#time_achieved = plan.planning_time()
	# 			#timestamp = self.dt/time_achieved
	# 			control_speed = plan.speed(0)
	# 			control_omega = plan.omega(0)
	# 			self.robot.move(control_speed, control_omega)
	# 			return plan
	# 		else:
	# 			return None
	
	# def simulate_loop(self, ball_observations):
	# 	plt.figure()
	# 	for ball_observation in ball_observations:
	# 		plan = self.simulate(ball_observation)
	# 		if plan is not None:
	# 			xs = [plan.x(t) for t in np.linspace(0, 1, 50)]
	# 			ys = [plan.y(t) for t in np.linspace(0, 1, 50)]
	# 			plt.plot(xs, ys, 'r--')
	# 			plt.scatter([ball_observation[0], self.robot.x], [ball_observation[1], self.robot.y])
	# 			plt.pause(3)
	# 	plt.show()


if __name__ == "__main__":
	pred_sim = PredictionSimulator([4, 2, np.pi/2], 0.2)
	pred_sim.simulate()
	
	#ball_observations = [[0, 1, 1], [0.3, 1.3, 2], [0.6, 1.6, 3], [0.9, 1.9, 4], [1.2, 2.2, 5], [1.5, 2.5, 6], [1.8, 2.8, 7], [2.1, 3.1, 8], [2.4, 3.4, 9], [2.7, 3.7, 10], [3.0, 4.0, 11], [3.3, 4.3, 12], [3.6, 4.6, 13], [3.9, 4.9, 14], [4.2, 5.2, 15]]
	#pred_sim.simulate_loop(ball_observations)
	
	# pred_sim.ball_predictor.record_observation([0, 1, 1]) 
	# pred_sim.ball_predictor.record_observation([0.3, 1.3, 2])
	# pred_sim.ball_predictor.fit()
	# plan = pred_sim.plan_to_intercept()
	# print("Planning time: ", plan.planning_time())
	# print("End of plan: ", plan.x(1), plan.y(1))
	# print("Where will ball be at planning time: ", pred_sim.ball_predictor.future_location(plan.planning_time()))
	#plt.scatter([0, 1], [1, 2])
	#plt.show()





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

