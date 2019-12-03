# turtlebot_soccer

To run the perception module run the following command: 

`roslaunch realsense2_camera rs_camera.launch filters:=pointcloud align_depth:=true`

and then in a new terminal window run the following command:

`rosrun segmentation main.py`

and then in another terminal window run the following command:

`roslaunch lab4_cam ar_track.launch`

You will need to have ar_maker_13 in the field of view of the camera. TODO needs a little more explanation.

To view stuff in rviz you need to add a camera and image from /camera/color/image_raw and then add a tf and a PointStamped. Set the fixed frame to ar_marker_13



