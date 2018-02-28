# Flags that control the conditional compilation of this library for use on different platforms. The
# defaults set here will be exported to dependent projects via catkin. Dependent projects can use
# cmake's `remove_definitions` command to change these defaults if desired.

#Include code that interfaces with ROS
add_definitions(-DROS)

#Control which platform's underlying networking library is used by this package.
#The package will not build unless one (and only one) of these definitions is set.
add_definitions(-DLINUXSOCKETS)
#add_definitions(-DMOTOPLUS)
