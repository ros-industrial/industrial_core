cmake_minimum_required(VERSION 3.0.2)

project(industrial_robot_simulator)

find_package(catkin REQUIRED)

catkin_package()

##########
## Test ##
##########

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

#############
## Install ##
#############


catkin_install_python(PROGRAMS industrial_robot_simulator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
