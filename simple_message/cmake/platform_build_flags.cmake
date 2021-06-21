# Flags that control the conditional compilation of this library for use on different platforms. The
# defaults set here will be exported to dependent projects via catkin. Dependent projects can use
# cmake's `remove_definitions` command to change these defaults if desired.

#Include code that interfaces with ROS
add_definitions(-DSIMPLE_MESSAGE_USE_ROS)

#Control which platform's underlying networking library is used by this package.
#The package will not build unless one (and only one) of these definitions is set.
add_definitions(-DSIMPLE_MESSAGE_LINUX)
#add_definitions(-DSIMPLE_MESSAGE_MOTOPLUS)

#--------------------------------------------------------------------------
#Old definitions (deprecated in kinetic, to be removed in lunar or melodic)
#--------------------------------------------------------------------------
#add_definitions(-DROS)
#add_definitions(-DLINUXSOCKETS)
#add_definitions(-DMOTOPLUS)
