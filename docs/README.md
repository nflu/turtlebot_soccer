# Project Overview

Imagine a robot playing soccer. Nearly every action requires the robot to first **intercept a rolling ball**. For example, interception is all that is required for defending a goal. To make a shot on goal or pass to another player the ball must be intercepted and then hit to a target location. 

## Autonomy Stack
We split our task into these steps. Our project is organized by having each of these blocks as a distinct ROS package.
<img src = "visuals/block_diagram.PNG"/>
1. To detect and track the ball with the RGB and aligned Depth image
2. Predict the velocity of the ball
3. Plan an interception point
4. Move the TurtleBot to the desired position

To have our TurtleBot be successful, our modules had to be efficient and accurate, as the ball moved quite fast. 


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
-Neil

<img src = "visuals/perception_gif.gif">

On the RGB image, we use color thresholding to generate a mask of all the magenta objects. To clean noise from the image, we filter the image using dilation and erosion. This is an example of what our mask outputs. 
<img src = "visuals/perception_mask.PNG"/>
It may seem like our threshold range is too narrow, but our detection rarely gives false positives and is robust to many similarly colored objects within the room, such as the purple chairs. 

We then find the minimum enclosing circle to generate our prediction of what is the ball in the image. The pixel location of the center of the circle is our estimate of the ball position (u,v). 
<img src = "visuals/perception_color_thresholding.PNG"/>
The yellow circle is our prediction of the ball location. The red contrail is made up of our last twenty predictions. 

To find how far away the ball is from the RealSense camera, we use the aligned depth image. We simply index into pixel location (u.v) of our ball position estimate to get the depth (w) at that point. 
<img src = "visuals/perception_depth.PNG"/>
We mapped the depths from the depth image onto a red to blue range of colors. The yellow circle is our prediction of the ball location. 

We decided to define everything within a world frame defined by an AR tag. To transform our prediction from a point within the camera frame to a point within the world frame, we solve for $X$, $Y$, and $Z$ in the following set of equations:

$$ \begin{bmatrix} u \\ v \end{bmatrix} = \frac{1}{w} \begin{bmatrix} u' \\ v' \end{bmatrix}$$ (1)
$$\begin{bmatrix} u' \\ v' \\ w \end{bmatrix} = \begin{bmatrix} f_x && s && x_0 \\ 0 && f_y && y_0 \\ 0 && 0 && 1 \end{bmatrix} \begin{bmatrix} X \\ Y \\ Z \end{bmatrix} $$ (2)

We also track our TurtleBot using an AR Tag. For the AR Tag tracking, we used what we did in Lab 4. 

<img src = "visuals/perception_rviz.PNG"/>
This rviz screen shows all the frames that we had. AR Marker 13 defines the world frame, the pink point is our prediction of where the soccer ball is, and AR Marker 4 is where the TurtleBot is. We can also see the camera optical frame relative to the world frame. 
-Angela

# Prediction



# Planning

# Control and Actuation

# Demos
<iframe class="video" src="https://www.youtube.com/watch?v=AVnXz0teLzA" frameborder="0" gesture="media" allow="encrypted-media" allowfullscreen></iframe>