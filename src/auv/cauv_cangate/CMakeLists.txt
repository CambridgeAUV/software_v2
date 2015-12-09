cmake_minimum_required(VERSION 2.8.3)
project(cauv_cangate)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

## Declare ROS messages and services
add_message_files(FILES msg_motor_demand.msg msg_pressure_status.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(cauv_cangate src/cauv_cangate.cpp)
target_link_libraries(cauv_cangate ${catkin_LIBRARIES})
add_dependencies(cauv_cangate cauv_cangate_generate_messages_cpp)
