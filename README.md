# GUI_navigation

### shell A (for camera)
$ cd GUI_navigation/catkin_ws_test/
$ catkin_make
$ source devel/setup.bash
$ roslaunch realsense2_camera rs_camera.launch align_depth:=true


### shell B
$ cd GUI_navigation/catkin_ws_test/
$ source devel/setup.bash
$ roslaunch test test_gui.launch

### shell C
$ cd GUI_navigation/catkin_ws_test/
$ source devel/setup.bash
$ rostopic echo /artifact_ses

