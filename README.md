# turtlebot_soccer

Please see our website [here](https://nflu.github.io/turtlebot_soccer/README)!

## Turtlebot

Put an AR tag on top of turtlebot and have the y direction of the AR point in the forward 
direction of the turtlebot. 

Remember to set 

`export ROS_MASTER_URI=http://[TurtleID].local:11311`

`ssh turtlebot@[TurtleId].local` password is `EE106A19`

then run

`roslaunch turtlebot_bringup minimal.launch --screen`

If you want to check things are working you can run teleop with:

`roslaunch turtlebot_teleop keyboard_teleop.launch --screen`

## Perception

Grab a big tripod instead of the small one that comes with the realsense.

Make sure you have the realsense plugged into a USB 3 port through a USB 3 cable.

To run the perception module run the following command: 

`roslaunch realsense2_camera rs_camera.launch align_depth:=true tf_prefix:=realsense`

`roslaunch lab4_cam ar_track.launch output_frame:=realsense_color_frame`

`rosrun perception main.py --verbosity 0`

You will need to have ar_maker_13 in the field of view of the camera. It uses 
this ar tag as the world frame and publishes the state estimate of the ball in 
this frame.

If you want to use a different ar tag use the `--ar_tag_number` argument:

`rosrun perception main.py --ar_tag_number {ar tag number}`

By default perception displays the RGB image, depth image, and mask and prints the 
location of the ball. To remove this for improved speed use the `--verbosity` 
argument see the [constructor of RealsensePerceptionProcess](https://github.com/nflu/turtlebot_soccer/blob/6b999f9d2ec10b91aaa965214fd81ab301d5ae08/src/segmentation/src/main.py#L51) for more info.

To view the ball location in rviz run 
`roslaunch turtlebot_rviz_launchers view_navigation.launch` then 

open the `turtlebot_soccer.rviz` configuration.

To check the speed of everything run 

`rostopic hz /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /state_estimate /predicted_path /intersection_point`

they should all be around 30 hz. Sometimes the camera will overheat and depth and/or RGB images will drop to 15 Hz. Unplug the camera and cool it down and then it should be back at 30 Hz.

## Prediction

`rosrun prediction main.py`


## Planning

`rosrun planning main.py --turtlebot_frame {turtlebot_frame}`

where `{turtlebot_frame}` is the frame of the turtlebot. 

### Interception
`rosrun turtlebot_control turtlebot_control.py {color} {turtlebot_frame}`

where `{color}` is the color of the turtlebot. 

### Move turtlebot to stationary ball

`rosrun turtlebot_control turtlebot_control_lab_4.py`
