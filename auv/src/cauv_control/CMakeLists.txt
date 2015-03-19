cmake_minimum_required(VERSION 2.8.3)
project(cauv_control)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(FILES msg_motorcontrol_demand.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${CATKIN_DEVEL_PREFIX})

#add_executable(cauv_priority_mgr src/cauv_priority_mgr.cpp)
#target_link_libraries(cauv_priority_mgr ${catkin_LIBRARIES})
#add_dependencies(cauv_priority_mgr cauv_priority_mgr_generate_messages_cpp)

#add_executable(cauv_motor_control_test_write src/cauv_motor_control_test_write.cpp)
#target_link_libraries(cauv_motor_control_test_write ${catkin_LIBRARIES})
#add_dependencies(cauv_motor_control_test_write cauv_motor_control_test_write_generate_messages_cpp)
