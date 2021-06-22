# The simple_message library is designed to cross compile on Ubuntu
# and various robot controllers. This requires conditionally compiling
# certain functions and headers. The default flags in this file enable
# compiling for a ROS node. This file is also exported to dependent packages
# via the catkin_package(..) commmand.

# Note: these are the same set of defines passed to target_compile_definitions(..)
# in the simple_message/CMakeLists.txt. In order to keep things simple and
# deterministic, they are replicated here.

# with plain CMake, we'd use target_compile_definitions(..) and then
# target_link_libraries(..) with EXPORT(..)-ed targets, but that's not possible
# right now with Catkin, so we use this work-around.

set(simple_message_DEFINITIONS         SIMPLE_MESSAGE_USE_ROS;SIMPLE_MESSAGE_LINUX)
set(simple_message_bswap_DEFINITIONS   SIMPLE_MESSAGE_USE_ROS;SIMPLE_MESSAGE_LINUX;BYTE_SWAPPING)
set(simple_message_float64_DEFINITIONS SIMPLE_MESSAGE_USE_ROS;SIMPLE_MESSAGE_LINUX;FLOAT64)
