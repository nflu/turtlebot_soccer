roslaunch realsense2_camera rs_camera.launch \
align_depth:=true tf_prefix:=realsense &
roslaunch lab4_cam ar_track.launch output_frame:=realsense_color_frame &
rosrun perception main.py
