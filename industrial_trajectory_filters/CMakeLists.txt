# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(industrial_trajectory_filters)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS moveit_ros_planning trajectory_msgs)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# Package definitions
set(MOVEIT_LIB_NAME moveit_industrial_trajectory_filters_plugins)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS moveit_ros_planning trajectory_msgs
    INCLUDE_DIRS include
    LIBRARIES ${MOVEIT_LIB_NAME}
)

###########
## Build ##
###########

add_library(${MOVEIT_LIB_NAME} src/n_point_filter.cpp src/uniform_sample_filter.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############
#install(TARGETS ${MOVEIT_LIB_NAME}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})




