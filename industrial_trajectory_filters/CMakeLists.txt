cmake_minimum_required(VERSION 3.0.2)

project(industrial_trajectory_filters)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  class_loader
  moveit_core
  moveit_ros_planning
  pluginlib
  trajectory_msgs
)

find_package(orocos_kdl REQUIRED)

catkin_package(
    CATKIN_DEPENDS
      class_loader
      moveit_core
      moveit_ros_planning
      pluginlib
      trajectory_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    DEPENDS orocos_kdl
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/add_smoothing_filter.cpp
  src/n_point_filter.cpp
  src/smoothing_trajectory_filter.cpp
  src/uniform_sample_filter.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES planning_request_adapters_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
