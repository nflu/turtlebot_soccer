from collections import deque
import numpy as np
import cv2
import imutils
from matplotlib.pylab import cm
 
# define the lower and upper boundaries of the purple ball RGB
rgb_lower = (63, 0, 52)
rgb_upper = (255, 55, 140)

hsv_lower = (0, 0, 0)
hsv_upper = (255, 255, 255)

# how many points to store in deque for trail of ball
n = 20
pts = deque(maxlen=n)


def pixel_to_point(u, v, w, cam_matrix):
    """
    converts a pixel at location u,v with depth w to a point in the camera frame
    using the fundamental matrix cam_matrix
    :param u: pixel location
    :param v: pixel location
    :param w: depth of pixel
    :param cam_matrix: fundamental matrix of camera
    :return:
    """
    u *= w
    v *= w
    x, y, z = np.linalg.solve(cam_matrix, np.array([u, v, w]))

    # for some reason the camera frame is a little different than the
    # fundamental matrix so we have to switch some axes
    return np.array([z, -x, -y])


def tracking(rgb, depth, depth_scale, cam_matrix, verbosity=0, use_hsv=False):
    """
    computes the position of the center of the ball in the camera frame
    :param rgb: rgb image numpy array
    :param depth: depth image numpy array. must be aligned with rgb image on
    realsense use align_depth:=true when starting ros node or use rs.align if
    using pyrealsense2
    :param depth_scale: conversion factor from depth value to distance
    :param cam_matrix: fundamental matrix of depth camera
    :param verbosity: Determines how much to print, display and save. Higher
    verbosity is better for debugging but slower. If set to 0 will only
    print error messages. If set to 1 will also print pixel location of ball
    center. If set to 2 will also display RGB and depth images with ball
    detection. If set to 3 will also save RGB, depth, blurred images and mask.
    :param use_hsv: if true converts image to hsv and uses hsv bounds
    :return: location of center of ball in camera frame
    """

    # converts from RGB to BGR because cv2 expects BGR
    rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

    # Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(rgb, (11, 11), 0)

    if use_hsv:
        image = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        lower = hsv_lower
        upper = hsv_upper
    else:
        image = blurred
        lower = rgb_lower
        upper = rgb_upper

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(image, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    center = None
    
    # only proceed if at least one contour was found
    if len(contours) > 0:
        # find the largest contour in the mask, then use it to compute the
        # minimum enclosing circle and centroid
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        moments = cv2.moments(c)
        center = (int(moments["m10"] / moments["m00"]),
                  int(moments["m01"] / moments["m00"]))

        # get distance to center and find location of point in camera frame
        distance = depth[int(y), int(x)] * depth_scale            
        
        # depth sensor reads 0.0 when it can't compute depth for pixel
        if distance == 0.0:
            point = None
        else:
            point = pixel_to_point(x, y, distance, cam_matrix)


        if verbosity >= 1:
            print(int(x), int(y))
            print(center)
            if distance != 0.0:
                print('center of detected object is ', distance, 'meters away')

        # convert depth to rgb so it will display colors well need to convert
        # before drawing circle and center below
        depth = np.uint8(cm.jet(depth)*255)
        depth = cv2.cvtColor(depth, cv2.COLOR_RGBA2BGR)

        # only proceed if the radius meets a minimum size
        if radius > 5:
            # draw the circle and centroid on the frame, then update the list
            # of tracked points
            cv2.circle(rgb, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(rgb, center, 5, (0, 0, 255), -1)
            cv2.circle(depth, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(depth, center, 5, (0, 0, 255), -1)
    else:
        point = None
    # update the points queue
    pts.appendleft(center)

    # loop over the set of tracked points
    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore them
        if pts[i - 1] is None or pts[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and draw the connecting
        # lines
        thickness = int(np.sqrt(n / float(i + 1)) * 2.5)
        cv2.line(rgb, pts[i - 1], pts[i], (0, 0, 255), thickness)
        cv2.line(depth, pts[i - 1], pts[i], (0, 0, 255), thickness)

    # convert depth to rgb so it will display colors well
    if len(contours) == 0:
        depth = np.uint8(cm.jet(depth)*255)
        depth = cv2.cvtColor(depth, cv2.COLOR_RGBA2BGR)

    if verbosity >= 2:
        cv2.imshow("RGB", rgb)
        cv2.imshow("Depth", depth)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)
    if verbosity >= 3:
        cv2.imwrite('rgb.png', rgb)
        cv2.imwrite('blurred.png', blurred)
        cv2.imwrite('image.png', image)
        cv2.imwrite('depth.png', depth)
    return point
