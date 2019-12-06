roslaunch realsense2_camera rs_camera.launch align_depth:=true &
roslaunch lab4_cam ar_track.launch &
rosrun perception main.py