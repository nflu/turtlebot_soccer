---
latex: yes
---

# Project Overview

Imagine a wheeled robot playing soccer. Nearly every action requires the robot to first intercept a rollingball. For example, interception is all that is required for defending a goal. To make a shot on goal or pass to another player the ball must be intercepted and then hit to a target location. In this project, we were able to have a TurtleBot perform that fundamental task of intercepting a rolling ball. We used computer vision to track where the ball was, prediction and planning to find a point for the TurtleBot to intercept the ball, and finally actuation on the TurtleBot to move the TurtleBot to the interception point. 

We split our task into these steps. Our project is organized by having each of these blocks as a distinct ROS package.
<img src = "visuals/block_diagram.PNG"/>
1. To detect and track the ball with the RGB and aligned Depth image
2. Predict the velocity of the ball
3. Plan an interception point
4. Move the TurtleBot to the desired position

To have our TurtleBot be successful, our modules had to be efficient and accurate, as the ball moved quite fast. 


# Perception

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

# Prediction



# Planning

# Control and Actuation

# Demos
<iframe class="video" src="https://www.youtube.com/watch?v=AVnXz0teLzA" frameborder="0" gesture="media" allow="encrypted-media" allowfullscreen></iframe>