cmake_minimum_required(VERSION 2.8.3)
project(ncsist_gui)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_definitions(-Wall)
add_definitions(-O0)
add_definitions(-g)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  subt_msgs
  cv_bridge
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5Widgets)   # Qt Environment


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ncsist_gui
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


qt5_wrap_ui(UIC src/mainwindow.ui)

add_executable(ncsist_gui  src/mainwindow.cpp src/mainwindow.ui src/main.cpp src/sub_thread.cpp src/sub_artimap.cpp )

target_link_libraries(ncsist_gui ${catkin_LIBRARIES} )
target_link_libraries(ncsist_gui ${catkin_LIBRARIES} Qt5::Widgets)

install(TARGETS ncsist_gui
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
