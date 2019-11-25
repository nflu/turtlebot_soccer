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
 
# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
hsv_green_lower = (29, 86, 6)
hsv_green_upper = (64, 255, 255)

rgb_green_lower = (50, 70, 40)
rgb_green_upper = (100,120,80)
n=20
pts = deque(maxlen=n)
use_hsv = True
greenLower = hsv_green_lower if use_hsv else rgb_green_lower
greenUpper = hsv_green_upper if use_hsv else rgb_green_upper


def tracking(frame):
	# resize the frame, blur it, and convert it to the HSV
	# color space
	frame = imutils.resize(frame, width=600)
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	if use_hsv:
		image = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	else:
		image = blurred

	# print(hsv[int(hsv.shape[0]/2)][int(hsv.shape[1]/2)])
	cv2.imwrite('tennis_ball_rgb.png',image)
	cv2.imwrite('tennis_ball_frame_rgb.png',frame)
 
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
		print(x, y)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
 	else:
 		x,y = None,None
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
 
	# show the frame to our screen
	#plt.imshow(frame)
	#plt.show()
	cv2.imshow("Frame", frame)
	cv2.waitKey(1)

	return x,y