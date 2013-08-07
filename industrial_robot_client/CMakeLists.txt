# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(industrial_robot_client)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs control_msgs trajectory_msgs simple_message actionlib_msgs actionlib urdf industrial_msgs industrial_utils)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system thread)

# The definition is copied from the CMakeList for the simple_message package.
add_definitions(-DROS=1)  #build using ROS libraries
add_definitions(-DLINUXSOCKETS=1)  #build using LINUX SOCKETS libraries

set(SRC_FILES src/joint_relay_handler.cpp
	      src/robot_status_relay_handler.cpp
              src/joint_trajectory_downloader.cpp
              src/joint_trajectory_streamer.cpp
              src/joint_trajectory_interface.cpp
              src/robot_state_interface.cpp
              src/utils.cpp)

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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

# NOTE: The libraries generated this package are not included in the catkin_package
# macro because libraries must be explicitly linked in projects that depend on this
# package.  If this is not done (and these libraries were exported), then multiple
# library definitions (normal - industrial_robot_client and byteswapped - 
# industrial_robot_client_bswap) are both included (this is bad).

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs control_msgs trajectory_msgs simple_message actionlib_msgs actionlib urdf industrial_msgs industrial_utils
    INCLUDE_DIRS include
)                


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# This library depends on the simple_message library, which is available
# in two formats to support endian differences between robot & PC.
#
# As in simple_message, two libraries are created: one that does direct
# passthrough of binary numeric data to the robot controller, and one that
# uses byte-swapping to account for endian mis-matches.
#
# Due to rosmake dependency limitations, higher-level code must explicitly
# link to the desired industrial_robot_client library:
#    target_link_libraries(my_node industrial_robot_client)
#      or
#    target_link_libraries(my_node industrial_robot_client_bswap)


add_library(industrial_robot_client ${SRC_FILES})
target_link_libraries(industrial_robot_client simple_message)

add_library(industrial_robot_client_bswap ${SRC_FILES})
target_link_libraries(industrial_robot_client_bswap simple_message_bswap)


# The following executables(nodes) are for applications where the robot
# controller and pc have the same byte order (i.e. byte swapping NOT
# required)

add_executable(robot_state
  src/generic_robot_state_node.cpp)
target_link_libraries(robot_state 
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})

add_executable(motion_streaming_interface
  src/generic_joint_streamer_node.cpp)
target_link_libraries(motion_streaming_interface 
  industrial_robot_client 
  simple_message
  ${catkin_LIBRARIES})

add_executable(motion_download_interface
  src/generic_joint_downloader_node.cpp)
target_link_libraries(motion_download_interface 
  industrial_robot_client 
  simple_message
  ${catkin_LIBRARIES})

# The following executables(nodes) are for applications where the robot
# controller and pc have different same byte order (i.e. byte swapping IS
# required)

add_executable(robot_state_bswap
  src/generic_robot_state_node.cpp)
target_link_libraries(robot_state_bswap 
  industrial_robot_client_bswap 
  simple_message_bswap
  ${catkin_LIBRARIES})

add_executable(motion_streaming_interface_bswap
  src/generic_joint_streamer_node.cpp)
target_link_libraries(motion_streaming_interface_bswap 
  industrial_robot_client_bswap  
  simple_message_bswap
  ${catkin_LIBRARIES})

add_executable(motion_download_interface_bswap
  src/generic_joint_downloader_node.cpp)
target_link_libraries(motion_download_interface_bswap 
  industrial_robot_client_bswap  
  simple_message_bswap
  ${catkin_LIBRARIES})

# The following executables(nodes) interface with the robot controller
# at a higher level so there is no need to create two versions (one with
# byte swapping, one without)

add_executable(joint_trajectory_action 
  src/generic_joint_trajectory_action_node.cpp
  src/joint_trajectory_action.cpp)
target_link_libraries(joint_trajectory_action 
  industrial_robot_client ${catkin_LIBRARIES})
add_dependencies(joint_trajectory_action industrial_robot_client_gencpp)

# Testing - Only performed on normal (non byte swapped library)
catkin_add_gtest(utest_robot_client test/utest.cpp)
target_link_libraries(utest_robot_client 
  industrial_robot_client 
  ${catkin_LIBRARIES})


#############
## Install ##
#############
install(
    TARGETS industrial_robot_client industrial_robot_client_bswap
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS 
        robot_state 
        robot_state_bswap 
        motion_streaming_interface 
        motion_download_interface 
        motion_streaming_interface_bswap 
        motion_download_interface_bswap 
        joint_trajectory_action 
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

foreach(dir config launch)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

