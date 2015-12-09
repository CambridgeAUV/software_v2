cmake_minimum_required(VERSION 2.8.3)
project(cauv_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  cauv_cangate
  cauv_utility
)

find_package(Boost REQUIRED COMPONENTS 
  system
  thread
)

add_message_files(
  FILES
  msg_floatYPR.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp message_runtime cauv_utility
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cauv_control src/control.cpp src/pid.cpp)

add_dependencies(cauv_control
  cauv_control_generate_messages_cpp
  cauv_cangate_generate_messages_cpp
)

target_link_libraries(cauv_control
  ${catkin_LIBRARIES}
  boost_system
  boost_thread
)