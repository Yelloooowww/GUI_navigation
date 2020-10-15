# Husky A* navigation with GUI

## Launch GUI on laptop
`laptop $ git clone https://github.com/Yelloooowww/GUI_navigation.git`

`laptop $ cd ~/GUI_navigation/catkin_ws/`

`laptop $ catkin_make`

`laptop $ source ~/GUI_navigation/catkin_ws/devel/setup.bash`

`laptop $ export ROS_MASTER_URI=http://192.168.1.11:11311`

`laptop $ export ROS_IP=192.168.1.101`

`laptop $ roslaunch ncsist_gui ncsist_gui.launch veh:=husky1`


## Launch Rviz on laptop
`laptop $ export ROS_MASTER_URI=http://192.168.1.11:11311`

`laptop $ export ROS_IP=192.168.1.101`

`laptop $ rviz`
