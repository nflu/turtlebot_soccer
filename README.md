# turtlebot_soccer

## Turtlebot

put an ar tag on top of turtlebot

Remember to set 

`export ROS_MASTER_URI=http://[TurtleID].local:11311`

`ssh turtlebot@[TurtleId].local` password is `EE106A19`

then run

`roslaunch turtlebot_bringup minimal.launch --screen`

and

`roslaunch turtlebot_bringup 3dsensor.launch`

then run 

`rosrun tf static_transform_publisher 0.014 0 -0.397 0 0 0 ar_marker_5 base_link 100`

or whichever ar tag is on the top of the turtlebot 

## Perception

Grab a big tripod instead of the small one that comes with the realsense.

Remember to set 

`export ROS_MASTER_URI=http://[TurtleID].local:11311`

Make sure you have the realsense plugged into a USB 3 port through a USB 3 cable

To run the perception module run the following command: 

`roslaunch realsense2_camera rs_camera.launch align_depth:=true`

and then in another terminal window run the following command:

`roslaunch lab4_cam ar_track.launch`

and then in a new terminal window run the following command:

`rosrun perception main.py`

You will need to have ar_maker_13 in the field of view of the camera. It uses 
this ar tag as the world frame and publishes the state estimate of the ball in 
this frame.

If you want to use a different ar tag use the `--ar_tag_number` argument:

`rosrun perception main.py --ar_tag_number {ar tag number}`

By default this displays the RGB image, depth image, and mask and prints the 
location of the ball. To remove this for improved speed use the `--verbosity` 
argument see the [constructor of RealsensePerceptionProcess](https://github.com/nflu/turtlebot_soccer/blob/6b999f9d2ec10b91aaa965214fd81ab301d5ae08/src/segmentation/src/main.py#L51) for more info.


To view the ball location in rviz run 
`roslaunch turtlebot_rviz_launchers view_navigation.launch` then 

1. add a camera and image from `/camera/color/image_raw`
2. add a TF only show the `camera_depth_optical_frame` and `ar_marker_13` (or whatever tag you use for world frame)
3. PointStamped from `/state_estimate`
4. Set the fixed frame to `camera_color_optical_frame` to get the camera working then change it to `ar_marker_13` (or whatever ar tag you use for world frame) for a more understandable view

To check the speed of everything run 

`rostopic hz /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /state_estimate `

they should all be around 30 hz.
