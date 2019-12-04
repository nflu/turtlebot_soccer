# turtlebot_soccer

Make sure you have the realsense plugged into a USB 3 port through a USB 3 cable

To run the perception module run the following command: 

`roslaunch realsense2_camera rs_camera.launch align_depth:=true`

and then in another terminal window run the following command:

`roslaunch lab4_cam ar_track.launch`

and then in a new terminal window run the following command:

`rosrun segmentation main.py`

You will need to have ar_maker_13 in the field of view of the camera. It uses 
this ar tag as the world frame and publishes the state estimate of the ball in 
this frame.

If you want to use a different ar tag use the `--ar_tag_number` argument:

`rosrun segmentation main.py --ar_tag_number {ar tag number}`

By default this displays the RGB image, depth image, and mask and prints the 
location of the ball. To remove this for improved speed use the `--verbosity` 
argument see the [constructor of RealsensePerceptionProcess](https://github.com/nflu/turtlebot_soccer/blob/6b999f9d2ec10b91aaa965214fd81ab301d5ae08/src/segmentation/src/main.py#L51) for more info.


To view the ball location in rviz you need to add a camera and image from /camera/color/image_raw and then add a tf and a PointStamped from /state_estimate. Set the fixed frame to ar_marker_13



