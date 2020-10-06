# GUI_navigation

### shell A (for camera)
$ roslaunch realsense2_camera rs_camera.launch align_depth:=true
or
$ roslaunch realsense2_camera rs_aligned_depth.launch 



### shell B
$ cd GUI_navigation/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch ncsist_gui start_ui.launch


