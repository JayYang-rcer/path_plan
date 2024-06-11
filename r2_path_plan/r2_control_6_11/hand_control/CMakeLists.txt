cmake_minimum_required(VERSION 3.0.2)
project(hand_control)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rc_msgs
  r2_msgs
  r2_app
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hand_control
 CATKIN_DEPENDS roscpp std_msgs 
#  LIBRARIES r2_app
#  DEPENDS system_lib
)


add_library(head_hand_src 
  include/${PROJECT_NAME}/hand_control.h
  src/hand_control.cpp
)
add_dependencies(head_hand_src ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(head_hand_src 
  ${catkin_LIBRARIES} 
)


add_executable(hand_control_node src/hand_control_node.cpp)
target_link_libraries(hand_control_node
  head_hand_src 
  ${catkin_LIBRARIES}
)

