cmake_minimum_required(VERSION 3.0.2)
project(r2_sensor)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  nav_msgs
  tf2_geometry_msgs
  r2_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES r2_sensor
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(sensor_msgs_server src/sensor_msgs_server.cpp)
target_link_libraries(sensor_msgs_server
  ${catkin_LIBRARIES}
)


add_executable(sensor_tf_pub src/sensor_tf_pub.cpp)
target_link_libraries(sensor_tf_pub
  ${catkin_LIBRARIES}
)


add_executable(sensor_tf_sub src/sensor_tf_sub.cpp)
target_link_libraries(sensor_tf_sub
  ${catkin_LIBRARIES}
)
