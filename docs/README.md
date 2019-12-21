---
latex: yes
---

# Project Overview

Imagine a robot playing soccer. Nearly every action requires the robot to first **intercept a rolling ball**. For example, interception is all that is required for defending a goal. To make a shot on goal or pass to another player the ball must be intercepted and then hit to a target location. 

The challenge from this project is to have working perception, prediction, planning, as well as control modules and then to integrate these modules into a full stack. Overall, our approach was one of a **feedback loop** involving this full stack in order to have the robot interact with the ball in real time and also so that our design would be robust to modeling errors. 

<img src = "https://neillugovoy.com/success_5.gif">

Thus, our project is broadly applicable to any robotic domain in a system must perform tasks in dynamic environments and in which vision is a main source of sensing. These environments can not only include moving objects such as the ball in our case, but also other agents, whose motion can also be predicted forward in time with some extension to our work. 


## Autonomy Stack
We split our task into these steps. Our project is organized by having each of these blocks as a distinct ROS package.
<img src = "https://neillugovoy.com/block_diagram.PNG"/>
1. To detect and track the ball
2. Predict the velocity of the ball
3. Plan an interception point
4. Move the TurtleBot to the desired position

To have our TurtleBot be successful, our modules had to be efficient and accurate, as the ball moved quite fast. 


# Perception

Using the RGB and depth images from a [Intel Realsense 435i camera](https://www.intelrealsense.com/depth-camera-d435i/) we estimated the ball's location.

## RGB Segmentation
We need to determine which pixels correspond to the ball so we applied a gaussian blur to smooth the edges and RGB thresholded to find the purple pixels.

### Problem
Other objects in the room like part of the chair cushions or clothing were in the purple range would cause **false positives**. Also sometimes due to sensor noise a random pixel would fall into the RGB range.

<img src = "https://neillugovoy.com/perception_purple.PNG"/>

### Solution
1. Since the ball is quite reflective and the room has bright lights the color varies dramatically from the top of the ball to the bottom. Instead of using an RGB range that fit the entire ball into the range we used a range that just fit the center of the ball where it is widest. 
<img src = "https://neillugovoy.com/perception_mask.PNG"/>

2. We performed erosion (removing pixels from boundary) on the mask and then dilation (adding pixels to boundary).  Erosion removes small connected components in the mask and thus eliminates small false positives. Since the RGB range was much tighter the false positives were sufficiently small to be eroded away.

From the mask we found all minimum enclosing circles of the connected components of hot pixels. Then we took the **largest** such circle. This is important because it means that all hot pixels on the ball will be enclosed and small false positives will be eliminated. Since the hot pixels are a strip where the ball is widest the circle fits to the ball quite nicely. 

<img src = "https://neillugovoy.com/perception_color_thresholding.PNG"/>
The yellow circle is our prediction of the ball location. The red contrail is made up of our last twenty predictions. 

Then we get the coordinates of the center pixel of that circle (u, v).
## Point Localization and Depth
If we assume that the image is generated via a pinhole camera model then the following equations describe the relationship between a point in 3D space (x,y,z) in the camera optical frame and the pixel (u,v) it maps to in the image.
<img src = "https://neillugovoy.com/pinhole_camera.png"/>

Where K is the camera matrix. Note that K is upper triangular with 1 in the bottom row 
<img src = "https://neillugovoy.com/k_matrix.png"/>

thus w=z which is the depth of (u,v) which we can get from the depth image! Thus we can construct (u',v',w') and solve the linear system for the point in the camera frame!

<img src = "https://neillugovoy.com/perception_rviz.PNG"/>
This rviz screen shows all the frames that we had. AR Marker 13 defines the world frame, the pink point is our prediction of where the soccer ball is, and AR Marker 4 is where the TurtleBot is. We can also see the camera optical frame relative to the world frame. 

### Problem
The depth image was very noisy for anything more than about 1.5 meters away. We observed variance of up to a meter on a single point in a stationary frame.

<!-- TODO insert noisy depth image -->

### Solution
1. We read about how the depth camera works. It uses an IR blaster and two IR cameras. In rooms with a lot of harsh lighting (such as the lab room) these sensors can be overexposed. We also read [this doc](https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/BKMs_Tuning_RealSense_D4xx_Cam.pdf) on tuning the Realsense and tuned the Realsense parameters for a couple hours until it worked better
2. The depth sensor will sometimes not be able to determine the depth of a point and will output 0. We set the state estimator to ignore these values instead of using them.
<img src = "https://neillugovoy.com/perception_depth.PNG"/>
We mapped the depths from the depth image onto a red to blue range of colors. The yellow circle is our prediction of the ball location. 

The perception works quite well and if it isn't displaying images will run as fast as it receives images from the camera.
<img src = "https://neillugovoy.com/perception_gif.gif">

# Prediction

Now that we had estimates of where the ball is we wanted to predict where it is going. We used a linear model of the ball in that we assumed that the ball moves at a constant speed and used the two most recent state estimates to calculate the velocity.

### Problem

There is noise in the position estimate. If the position estimate of the ball moves just a centimeter when it is stationary the velocity estimate will be 0.3 m/s (assuming it runs at 30 Hz). Thus even for a stationary ball we noticed non-negligible velocity estimates in random directions and when moving the velocity estimate would vary dramatically.

### Solution

From a signal processing perspective taking a numerical derivative of a noisy signal amplifies high frequency noise. Thus we implemented a moving average which is a low-pass filter. Intuitively with a noisy signal the variance is high but its mean is close to the true signal assuming the noise has a near zero mean. 

### New Problem

This adds a tunable parameter of how big to make the window for the average. A larger window results in lower noise estimates but it will be using older (potentially unrepresentative) data. If the ball suddenly goes out of frame, moves and then comes back in the velocity will be wildly inaccurate. Another downside is that the window must fill up with values before it can make velocity estimates, thus there will be some lag between the first position estimate and the first velocity estimate.

### New Solution

We tuned the size of the window to improve this tradeoff and also set a cutoff for how old of state estimates we would use. Thus if we didn't receive state estimates for a while we would throw out old samples. To fill up the window with values we started the ball in the frame before rolling it.

<img src = "https://neillugovoy.com/prediction.gif">

The blue sphere is the predicted location of the ball in 1 second.


# Planning

The goal of this module is to find the point of interception of the ball and robot. 

If we assume that the ball and robot are both points that move at a constant speed and we simply choose the direction of the robot (and hence the interception point), the point can be found by [solving a system of equations.](https://jaran.de/goodbits/2011/07/17/calculating-an-intercept-course-to-a-target-with-constant-direction-and-velocity-in-a-2-dimensional-plane/)

<img src = "https://neillugovoy.com/exact_interception.png">

### Problem
This method seeks to find a point where the two points intersect exactly thus sometimes solutions would be up to 400 meters away. This is because the turtlebot would barely miss the ball if meeting it close so it has to chase it and catch up far away. Additionally this method is very sensitive and due to noise in the positions and velocities the solution point would jump around frequently. 

<img src = "https://neillugovoy.com/far_away_interception.png">

### Solution
Instead of finding an exact interception we want to find a point where the two objects are sufficiently close. Additionally we want to choose the closest and lowest control solution rather than going far away and using a lot of effort to get a slightly better solution.

Thus we just simulated where the ball would be at times within 1 second and checked how close the turtlebot could get going at a constant speeds and picked the earliest and lowest speed point. Because new predictions were constantly being generated for new measurements, the robot would be constantly replanning in response to new information about where the ball was and where it was going. 

<img src = "https://neillugovoy.com/planning.png">

In this diagram (not to scale) we see the different points the ball will be at various times and the radius of acceptable interception distance around each point. The dotted line indicates where the robot will be.

### New Problem
If we assume that the TurtleBot could move at one speed, the interception point will change to a different point that is further away. This is because we assume that the Turtlebot can only move at one speed and not slower, so as we approach the soccer ball the point moves.

<img src = "https://neillugovoy.com/bad_interception.gif">

In this gif you can see that the TurtleBot approaches the soccer ball, then backs up and turns, because we input a different interception point. 

### New Solution
We assume that the TurtleBot moves in a straight line now at various nominal speeds. Then we iterate through the speeds, from slowest to fastest, and find the first point where the distance between the TurtleBot and soccer ball is within our desired threshold. 


```
Pseudocode:

Given: predict_ball(t) from prediction module
speeds = [set of possible speeds for the robot]
times = [times from 0 to 1 second]
turtlebot_pt = current location of the robot

for t in times:
    ball_pt = predict_ball(t)
    movement_vector = 
        normalize(ball_pt - turtlebot_pt)
    for s in speeds:
        turtlebot_next_pt = 
            turtlebot_pt + (movement_vector * s * t)
        dist = distance(turtlebot_next_pt, ball_pt)
        if dist < epsilon:
            return turtlebot_next_pt
```

<img src = "https://neillugovoy.com/planning.gif">

You can see in this gif the planned interception point in green. While the robot is near the ball the point is near both of them, but once it goes past the robot the interception point is far away. This was an early iteration of the controller. See next section for details.

# Control and Actuation

Given the interception point outputted by our planning module, we implemented a simple proportional controller to move the robot to that point as quickly as possible. The controller gives the robot a linear and angular velocity control command. We then had to tune our gains accordingly.

### Problem
With a proportional controller, the Turtlebot would slow down dramatically when it got close to the ball.

<img src = "https://neillugovoy.com/proportional_control.gif">

### Solution
To make our controller act more aggressively, we put our error through an arctan function. This would make our controller act more like a smoothed bang-bang controller, because the Turtlebot will be moving close to full speed at distances far away from the ball.

<img src = "https://neillugovoy.com/arctan.png">

### New Problem
Due to the Turtlebot's two-wheel design and aggressive control inputs, the Turtlebot had trouble going forward and turning at the same time. For this reason, our robot couldn't perform maneuvers necessary for certain interception scenarios.

<img src = "https://neillugovoy.com/turn_too_long.gif">


### New Solution
We clipped the control inputs to be within the range where the turtlebot could still turn while moving forward. We also increased the frequency of the controller from 10 Hz to 30 Hz so that the controller could perform fine adjustments faster to compensate for being more aggressive.

### Dealing with Non-holonomic constraints

Initially we tried to use a spline planner that took into account the fact that the turtlebot can only move along the axis it is pointing. However we found that this planner did not work well in practice. You can see a gif of it in simulation below. Thus instead we just use velocities lower than the actual top speed of the turtlebot so time spent turning would be accounted for. We could have added more complexity about the actual velocity the turtlebot can travel but in general we found the planner to be too pessismistic rather than optimistic.

<img src = "https://neillugovoy.com/spline_simulation.gif">

Here, the purple dots are measurements of the ball and each black dot is the robot at a successive time step as it moves along the spline path (red dashed curve). 

# Demos
<iframe width="560" height="315" src="https://www.youtube.com/embed/Avs59YVElMI" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

<iframe width="560" height="315" src="https://www.youtube.com/embed/AVnXz0teLzA" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

<iframe width="560" height="315" src="https://www.youtube.com/embed/13Bkve3_Uo4" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

<img src = "https://neillugovoy.com/success_1.gif">
<img src = "https://neillugovoy.com/success_2.gif">
<img src = "https://neillugovoy.com/success_3.gif">


# Results and Conclusion
In spite of challenges such as a bumpy floor that caused the ball to curve and the speed limitations of the Turtlebot, we were able to achieve our initial design goal. As shown in our videos, we were able to have successful interceptions in challenging scenarios up to 7 times consecutively. All modules were running at 30 Hz, which is as fast as the RealSense publishes. Additionally, overall we found that the combination of our simple prediction, planning, and control modules and the global feedback loop worked very well for our application. Anything that these simplicities could not capture was caught by the closed loop.  

Extensions to our work would include using the Turtlebot's onboard camera to improve our estimate of the ball's location and planning to hit the ball to a desired location instead of simple interception. 


# Team
### Angela Wang
Angela is a fourth year EECS major interested in signals and systems. She does robotic simulation research with Professor Ron Fearing and communication network research with Professor Shyam Parekh. She contributed to all of the perception, planning, prediction, and control modules. 
### Khadijah Flowers
Khadijah is a fourth year CS major. She helped with the prediction and planning modules.
### Neil Lugovoy
Neil is a fourth year CS major who does research in Professor Claire Tomlin's lab on safe learning in robotics. In this project, he contributed to all of the perception, planning, prediction, and control modules and also worked heavily on integration. 
### Sampada Deglurkar
Sampada is a fourth year EECS major. She worked mainly on the prediction, planning, and control sides of the project and also helped with integration.
