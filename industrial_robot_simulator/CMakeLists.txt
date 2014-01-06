cmake_minimum_required(VERSION 2.8.3)

project(industrial_robot_simulator)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs control_msgs trajectory_msgs industrial_robot_client)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs control_msgs trajectory_msgs industrial_robot_client
)


install(PROGRAMS industrial_robot_simulator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
