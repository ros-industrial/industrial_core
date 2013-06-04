# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(industrial_msgs)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS std_msgs trajectory_msgs genmsg message_generation)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DebugLevel.msg
  DeviceInfo.msg
  RobotMode.msg
  RobotStatus.msg
  ServiceReturnCode.msg
  TriState.msg)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CmdJointTrajectory.srv
  GetRobotInfo.srv
  SetDrivePower.srv
  SetRemoteLoggerLevel.srv
  StartMotion.srv
  StopMotion.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES trajectory_msgs std_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs trajectory_msgs genmsg
)

