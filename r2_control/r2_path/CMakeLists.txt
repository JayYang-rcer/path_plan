cmake_minimum_required(VERSION 3.0.2)
project(r2_path)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  rc_msgs
  r2_msgs
  message_generation
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES r2_path
 CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime 
#  DEPENDS system_lib
)


add_library(head_path_src
  include/${PROJECT_NAME}/path_base.h
  src/path_plan/path_base.cpp
)
add_dependencies(head_path_src ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(head_path_src
  ${catkin_LIBRARIES}
)


add_library(head_math_src
  include/${PROJECT_NAME}/math.h
  src/tool/math.cpp
)
add_dependencies(head_math_src ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(head_math_src
  ${catkin_LIBRARIES}
)


add_library(head_pid_src
  include/${PROJECT_NAME}/pid.h
  src/tool/pid.cpp
)
add_dependencies(head_pid_src ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(head_pid_src head_math_src
  ${catkin_LIBRARIES}
)


add_library(head_path_run_src
  include/${PROJECT_NAME}/path_run.h
  src/path_run/path_run.cpp
)
add_dependencies(head_path_run_src ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(head_path_run_src 
  ${catkin_LIBRARIES}
)


add_executable(accel_pub src/velocity/accel_pub.cpp)
target_link_libraries(accel_pub
  ${catkin_LIBRARIES}
)


add_executable(take_ball_pub_test src/path_plan/take_ball_pub_test.cpp)
target_link_libraries(take_ball_pub_test
  ${catkin_LIBRARIES}
)


add_executable(path_run_node src/path_run/path_run_node.cpp)
add_dependencies(path_run_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_run_node
  head_path_run_src head_path_src head_pid_src head_math_src 
  ${catkin_LIBRARIES}
)