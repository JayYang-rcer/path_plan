cmake_minimum_required(VERSION 3.0.2)
project(r2_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  nav_msgs
  geometry_msgs
  std_msgs  
  nav_msgs
  actionlib
  actionlib_msgs
)


add_action_files(
  FILES
  pid_follower.action
)


## 配置 srv 源文件
add_service_files(
  FILES
  action_cmd.srv
  global_path.srv
)


add_message_files(
  FILES
  path_cmd.msg
  pid_info.msg
  controller_cmd.msg
  stm32.msg
  path_status.msg
  decision.msg
)


# 生成消息时依赖于 std_msgs
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
)


catkin_package(
 CATKIN_DEPENDS 
 message_runtime
 roscpp 
 std_msgs
)

