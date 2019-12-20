# Project Overview

Imagine a robot playing soccer. Nearly every action requires the robot to first **intercept a rolling ball**. For example, interception is all that is required for defending a goal. To make a shot on goal or pass to another player the ball must be intercepted and then hit to a target location. 

In this project, we were able to have a TurtleBot perform that fundamental task of intercepting a rolling ball. We used computer vision to track where the ball was, prediction and planning to find a point for the TurtleBot to intercept the ball, and finally actuation on the TurtleBot to move the TurtleBot to the interception point. 

## Autonomy Stack
<img src = "visuals/block_diagram.PNG"/>

# Perception

Using the RGB and depth images from a [Intel Realsense 435i camera](https://www.intelrealsense.com/depth-camera-d435i/) we estimated the ball's location.

## RGB Segmentation
We need to determine which pixels correspond to the ball so we applied a gaussian blur to smooth the edges and RGB thresholded to find the purple pixels.

TODO insert screenshot of thresholder tool

### Problem
Other objects in the room like part of the chair cushions or clothing were in the purple range would cause **false positives**. Also sometimes due to sensor noise a random pixel would fall into the RGB range.

TODO insert image with chair cushion

### Solution
1. Since the ball is quite reflective and the room has bright lights the color varies dramatically from the top of the ball to the bottom. Instead of using an RGB range that fit the entire ball into the range we used a range that just fit the center of the ball where it is widest. TODO insert image of mask
2. We performed erosion (removing pixels from boundary) on the mask and then dilation (adding pixels to boundary).  Erosion removes small connected components in the mask and thus eliminates small false positives. Since the RGB range was much tighter the false positives were sufficiently small to be eroded away.

From the mask we found all minimum enclosing circles of the connected components of hot pixels. Then we took the **largest** such circle. This is important because it means that all hot pixels on the ball will be enclosed and small false positives will be eliminated. Since the hot pixels are a strip where the ball is widest the circle fits to the ball quite nicely. 
TODO insert gif of mask and circle

Then we get the coordinates of the center pixel of that circle $(u, v)$.
## Point Localization and Depth
If we assume that the image is generated via a pinhole camera model then the following equations describe the relationship between a point in 3D space $(x,y,z)$ in the camera optical frame and the pixel $(u,v)$ it maps to in the image.
$$\begin{bmatrix} u'\\v'\end{bmatrix}=w\begin{bmatrix}u \\ v\end{bmatrix} $$
$$\begin{bmatrix}u'\\v'\\w\end{bmatrix} = K \begin{bmatrix}x\\y\\z\end{bmatrix} $$
Where $K$ is the camera matrix. Note that $K$ is upper triangular with $1$ in the bottom row 
$$K = \begin{bmatrix} f_x & f_y & x_0\\ 0 & f_y & y_0\\ 0& 0 & 1
			\end{bmatrix} $$
thus $w=z$ which is the depth of $(u,v)$ which we can get from the depth image! Thus we can construct $[u', v', w]^{T}$ and solve the linear system for $[x,y,z]^{T}$!

## Problem
The depth image was very noisy for anything more than about 1.5 meters away. We observed variance of up to a meter on a single point in a stationary frame.

TODO insert noisy depth image

## Solution
1. We read about how the depth camera works. It uses an IR blaster and two IR cameras. In rooms with a lot of harsh lighting (such as the lab room) these sensors can be overexposed. We also read [this doc](https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/BKMs_Tuning_RealSense_D4xx_Cam.pdf) on tuning the Realsense and tuned the Realsense parameters for a couple hours until it worked better
2. The depth sensor will sometimes not be able to determine the depth of a point and will output 0. We set the state estimator to ignore these values instead of using them.

TODO include picture of depth from perception video 

The perception works quite well and if it isn't displaying images will run as fast as it receives images from the camera.

# Prediction

# Planning

# Control and Actuation

# Demos
