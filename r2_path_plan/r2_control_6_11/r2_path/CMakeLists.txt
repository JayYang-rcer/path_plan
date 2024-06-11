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
  nav_msgs
  message_generation
)


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
  include/${PROJECT_NAME}/path_plan/path_base.h
  src/path_plan/path_base.cpp
)
add_dependencies(head_path_src ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(head_path_src
head_math_src
  ${catkin_LIBRARIES}
)


add_library(head_tracker_src
  include/${PROJECT_NAME}/trackers/pid_follower.h
  src/trackers/pid_follower.cpp
  src/trackers/pid_follower_config.cpp
)
add_dependencies(head_tracker_src ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(head_tracker_src
  ${catkin_LIBRARIES}
)


add_library(head_filter_src
  include/${PROJECT_NAME}/tool/filter.h
  src/tool/filter.cpp
)
add_dependencies(head_filter_src ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(head_filter_src
  ${catkin_LIBRARIES}
)


add_library(head_math_src
  include/${PROJECT_NAME}/tool/math.h
  src/tool/math.cpp
)
add_dependencies(head_math_src ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(head_math_src
  ${catkin_LIBRARIES}
)


add_library(head_pid_src
  include/${PROJECT_NAME}/tool/pid.h
  src/tool/pid.cpp
)
add_dependencies(head_pid_src ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(head_pid_src head_math_src head_filter_src
  ${catkin_LIBRARIES}
)


add_executable(accel_pub src/velocity/accel_pub.cpp)
target_link_libraries(accel_pub
  ${catkin_LIBRARIES}
)


add_executable(pid_follower src/trackers/pid_follower.cpp)
target_link_libraries(pid_follower
head_tracker_src head_pid_src
  ${catkin_LIBRARIES}
)

add_executable(path_app src/App/path_app.cpp)
target_link_libraries(path_app
head_tracker_src head_pid_src head_path_src
  ${catkin_LIBRARIES}
)

