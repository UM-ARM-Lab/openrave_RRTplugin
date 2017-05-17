cmake_minimum_required(VERSION 2.8.3)
project(openrave_rrtplugin)

set(CATKIN_PACKAGES
    openrave_catkin
    roscpp
    victor_hardware_interface
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES})

#find_package(catkin REQUIRED)
#find_package(catkin REQUIRED COMPONENTS
#  mps_msgs
#  roscpp
#  rospy
#  tf
#)
#find_package(openrave REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES or_controllers
    CATKIN_DEPENDS ${CATKIN_PACKAGES}
#    DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O0 -g -Wall -Wextra -Wpedantic -Wconversion")

## Declare a C++ library

# Stub library that registers the plugins with OpenRAVE.
openrave_plugin(${PROJECT_NAME}_plugin
#    include/openrave_rrtplugin/or_rrt_plugin.h
    include/openrave_rrtplugin/orplugin.h
#    include/openrave_rrtplugin/rrtHelpler.h
#    include/openrave_rrtplugin/rrtHelpler.hpp
    include/rrtHelpler.h
#    src/or_rrt_plugin.cpp
    src/orplugin.cpp
#    src/rrtHelper.h
    src/rrtHelper.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin
#    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#FILE(GLOB hpp_SRC "include/${PROJECT_NAME}/*.hpp")
#FILE(GLOB h_SRC "include/${PROJECT_NAME}/*.h")
#FILE(GLOB h_SRC "include/rrtHelper.h")
#FILE(GLOB cpp_SRC "src/*.cpp")
#FILE(GLOB py_SRC "scripts/*.py")


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#install(PROGRAMS
#    scripts/maze_2grippers.py
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.h"
#    PATTERN ".svn" EXCLUDE
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
#     # myfile1
#     # myfile2
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

