cmake_minimum_required(VERSION 2.8.3)
project(cauv_priority_mgr)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")
endif()

## Find catkin and any catkin packages
#find_package(catkin REQUIRED COMPONENTS roscpp roslib rospy std_msgs genmsg)

## Declare ROS messages and services
#add_service_files(FILES srv_control_command.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${CATKIN_DEVEL_PREFIX})

#add_executable(cauv_priority_mgr src/cauv_priority_mgr.cpp src/mgr_requeststack.cpp)
#target_link_libraries(cauv_priority_mgr ${catkin_LIBRARIES})
#add_dependencies(cauv_priority_mgr cauv_priority_mgr_generate_messages_cpp)
