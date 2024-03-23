cmake_minimum_required(VERSION 3.0.2)
project(r2_hardware)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  r2_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES r2_hardware
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


add_library(head_src
  include/${PROJECT_NAME}/communica.h
  src/uart/communica.cpp
)
add_dependencies(head_src ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(head_src
  ${catkin_LIBRARIES}
)


add_executable(communicate_node src/uart/communicate_node.cpp)
add_dependencies(communicate_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(communicate_node
  head_src
  ${catkin_LIBRARIES}
)


add_executable(pub_process src/uart/pub_process.cpp)
add_dependencies(pub_process ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_process
  head_src
  ${catkin_LIBRARIES}
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
