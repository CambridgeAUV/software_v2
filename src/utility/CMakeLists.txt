# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

cmake_minimum_required(VERSION 2.8.3)
project (cauv_utility)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS "include"
    LIBRARIES cauv_utility
    DEPENDS Boost
)

include_directories(include
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

add_library(cauv_utility
    STATIC
    math.cpp
    string.cpp
    uid.cpp
    time.cpp
    ratelimit.cpp
    random.cpp
    coordinates.cpp
    daemon.cpp
    files.cpp
    options.cpp
)

install(TARGETS cauv_utility
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
