import numpy as np
import matplotlib.pyplot as plt


def fit(two_points):
	point1 = two_points[0]
	point2 = two_points[1]
	
	A_x = np.array([[point1[2], 1], [point2[2], 1]])
	b_x = np.array([point1[0], point2[0]])
	[m1, b1] = np.linalg.solve(A_x, b_x)
	
	A_y = np.array([[point1[2], 1], [point2[2], 1]])
	b_y = np.array([point1[1], point2[1]])
	[m2, b2] = np.linalg.solve(A_y, b_y)

	return ([m1, b1], [m2, b2])


def line(x, m1, m2, b1, b2):
	return (m2/m1) * (x - b1) + b2


def plan(initial_state, final_state, t):
	"""
	This code computes and plot a third-order spline for a Dubins car from a starting state, non-zero speed to a goal state, non-zero speed. 
	"""

	# Initial and final speeds
	initial_speed = 10
	final_speed = 1

	# Times at which the Spline is plotted
	time_samples = 100
	times = np.linspace(0., t, time_samples)

	# Compute coefficients
	# ds are decided by the initial position
	d1 = initial_state[0]
	d2 = initial_state[1]

	# cs are dependent on the initial speed
	c1 = initial_speed * np.cos(initial_state[2])
	c2 = initial_speed * np.sin(initial_state[2])

	# Final state and speed puts some constraints on the remaining coefficients
	a1 = final_speed * np.cos(final_state[2]) - 2*final_state[0] + c1 + 2*d1
	a2 = final_speed * np.sin(final_state[2]) - 2*final_state[1] + c2 + 2*d2
	b1 = final_state[0] - a1 - c1 - d1
	b2 = final_state[1] - a2 - c2 - d2

	# Compute the state trajectory at time steps using the above co-efficients
	xs = np.zeros(time_samples)
	ys = np.zeros(time_samples)
	xs_dot = np.zeros(time_samples)
	ys_dot = np.zeros(time_samples)
	freq = 5

	for i in range(time_samples):
	  t = times[i]
	  xs[i] = t*t*(a1*t + b1) + c1*t + d1
	  ys[i] = t*t*(a2*t + b2) + c2*t + d2
	  xs_dot[i] = 3*t*t*a1 + 2*t*b1 + c1
	  ys_dot[i] = 3*t*t*a2 + 2*t*b2 + c2
	 
	speed = np.sqrt(xs_dot**2 + ys_dot**2)
	speed_angle = np.arctan2(ys_dot, xs_dot)

	# Let's plot the spline
	fig = plt.figure()
	ax = fig.add_subplot(111)

	ax.plot(xs, ys)
	ax.plot(final_state[0], final_state[1], 'ro')
	ax.plot(initial_state[0], initial_state[1], 'bo')
	# ax.quiver(xs[::freq], ys[::freq], xs_dot[::freq]/speed[::freq], ys_dot[::freq]/speed[::freq], units='width')
	ax.quiver(initial_state[0], initial_state[1], np.cos(initial_state[2]), np.sin(initial_state[2]), units='width')
	ax.quiver(final_state[0], final_state[1], np.cos(final_state[2]), np.sin(final_state[2]), units='width')

	ax.set_xlabel('x')
	ax.set_ylabel('y')

	ax.set_xlim(-0.5, 2.5)
	ax.set_ylim(-0.5, 2.5)

	fig = plt.figure()
	ax1 = fig.add_subplot(121)
	ax1.plot(times, speed, 'r--')
	ax1.set_xlabel('time')
	ax1.set_ylabel('v')

	ax2 = fig.add_subplot(122)
	ax2.plot(times, speed_angle, 'r--')
	ax2.set_xlabel('time')
	ax2.set_ylabel('theta')

	plt.figure()
	plt.plot(times, xs)
	plt.figure()
	plt.plot(times, ys)

	plt.show()

	reachable = False

	for i in range(len(xs)):
		if xs[i] == final_state[0] and ys[i] == final_state[1]:
			reachable = True

	if reachable:
		print(xs)
		print(ys)
		return xs, ys, speed_angle
	else:
		print(xs)
		print(ys)
		return reachable


if __name__ == "__main__":
	two_points = [[0, 0, 1], [4, 6, 6]]
	([m1, b1], [m2, b2]) = fit(two_points)
	x = np.linspace(-5, 10, 150)
	plt.plot(x, line(x, m1, m2, b1, b2))

	t = 7
	at_t_x = m1 * t + b1
	at_t_y = m2 * t + b2

	plt.scatter([0, 4, at_t_x], [0, 6, at_t_y])


	plt.show()

	# Initial and final states
	initial_state = np.array([0., 0., 0.])
	final_state = np.array([2., 2., 0.5*np.pi])
	t = 1.1

	val = plan(initial_state, final_state, t)

	if val == False: print(val)

