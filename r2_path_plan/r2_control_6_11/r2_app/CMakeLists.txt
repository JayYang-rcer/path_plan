cmake_minimum_required(VERSION 3.0.2)
project(r2_app)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  r2_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES r2_app
 CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  include/r2_app/take_ball.h
  src/take_ball.cpp
)

target_link_libraries(r2_app
  ${catkin_LIBRARIES}
)