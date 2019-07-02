cmake_minimum_required(VERSION 3.0.2)

project(industrial_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs trajectory_msgs)

add_message_files(
  FILES
  DebugLevel.msg
  DeviceInfo.msg
  RobotMode.msg
  RobotStatus.msg
  ServiceReturnCode.msg
  TriState.msg)

add_service_files(
  FILES
  CmdJointTrajectory.srv
  GetRobotInfo.srv
  SetDrivePower.srv
  SetRemoteLoggerLevel.srv
  StartMotion.srv
  StopMotion.srv)

generate_messages(
  DEPENDENCIES std_msgs trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs trajectory_msgs
)
