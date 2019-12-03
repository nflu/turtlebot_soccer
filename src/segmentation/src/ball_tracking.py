# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import matplotlib
from matplotlib import pyplot as plt
from matplotlib.pylab import cm
 
# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
hsv_green_lower = (29, 86, 6)
hsv_green_upper = (64, 255, 255)

rgb_green_lower = (63, 0, 52)
rgb_green_upper = (255,55,140)
n=20
pts = deque(maxlen=n)
use_hsv = False
greenLower = hsv_green_lower if use_hsv else rgb_green_lower
greenUpper = hsv_green_upper if use_hsv else rgb_green_upper

def pixel_to_point(u,v,w,cam_matrix):
	u*=w
	v*=w
	x,y,z = np.linalg.solve(cam_matrix, np.array([u,v,w]))
	return np.array([z,-x,-y])

def tracking(frame, depth, depth_scale, cam_matrix):
	# resize the frame, blur it, and convert it to the HSV
	# color space
	# print('frame shape:',np.shape(frame))
	# print('depth shape:',np.shape(depth))
	print('start of tracking')
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # TODO does this 
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	if use_hsv:
		# TODO and this line make sense? shouldn't it be RGB2HSV?
		image = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	else:
		image = blurred

	# print(hsv[int(hsv.shape[0]/2)][int(hsv.shape[1]/2)])
	cv2.imwrite('image.png',image)
	cv2.imwrite('frame.png',frame)
	cv2.imwrite('blurred.png',blurred)
 
	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(image, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)


	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None
 	
	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		print(int(x), int(y))
		print(center)
		distance = depth[int(y),int(x)] * depth_scale
		print('center of detected object is ', distance, 'meters away')
		depth = np.uint8(cm.jet(depth)*255)
		depth = cv2.cvtColor(depth, cv2.COLOR_RGBA2BGR)
		point = pixel_to_point(x, y, distance, cam_matrix)
		print('point:', point)
		# only proceed if the radius meets a minimum size
		if radius > 5:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
			cv2.circle(depth, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(depth, center, 5, (0, 0, 255), -1)
 	else:
 		point = None
	# update the points queue
	pts.appendleft(center)

	
	# loop over the set of tracked points
	for i in range(1, len(pts)):
		# if either of the tracked points are None, ignore
		# them
		if pts[i - 1] is None or pts[i] is None:
			continue
 
		# otherwise, compute the thickness of the line and
		# draw the connecting lines
		thickness = int(np.sqrt(n / float(i + 1)) * 2.5)
		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
		cv2.line(depth, pts[i - 1], pts[i], (0, 0, 255), thickness)

 
	# show the frame to our screen
	#plt.imshow(depth)
	#plt.show()
	if len(cnts) == 0:
		depth = np.uint8(cm.jet(depth)*255)
		depth = cv2.cvtColor(depth, cv2.COLOR_RGBA2BGR)
	cv2.imwrite('depth.png',depth)
	print('depth new shape:', np.shape(depth))
	cv2.imshow("Frame", frame)
	# depth = cv2.applyColorMap(cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY), cv2.COLORMAP_AUTUMN)
	cv2.imshow("Depth", depth)
	cv2.imshow("Mask", mask)
	cv2.waitKey(1)
	return point