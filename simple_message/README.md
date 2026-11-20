## Protocol

The Simple Message (!SimpleMessage) protocol defines the message
structure between the ROS driver layer and the robot controller itself.

Refer to
[REP-I0006: Message Structures of the ROS-Industrial Simple Message Protocol](https://github.com/ros-industrial/rep/blob/master/rep-I0006.rst)
for a description of the protocol.

## Typed Messages

The message protocol allows for an arbitrary data payload for message
and communications types. However, the client/server model requires that
both understand the data payload associated with the different message
and communications types. The typed message class enforces the data
payload structure. The typed message base class provides methods for
creating topic, reply, and request messages. If used in both the client
and server, the developer need not understand the structure of the data
payload. Unfortunately, a typical robot controller cannot use C++
classes, and thus the developer must understand the message protocol and
payload data structure in order to parse it on the robot controller
side. The documentation on message specific structures can be found in
the source header files. For convenience the message structure is also
shown here for common message types. For a more detailed example of a
typed message structure and how it is used see the following
[tutorial](Industrial/Tutorials/create_joint_position_streaming_interface_using_tcp_socket_libraries).

## Generic Connections

The simple message utilizes a abstract connection (!SmplMsgConnection)
interface to send messages to the industrial robot controller. The
interface makes two assumptions: 1. The connection is capable of sending
raw bytes 1. The data connection has an explicit connect that
establishes the connection (and an associated disconnect method). NOTE:
For data connections that are connectionless, such as UDP, the
connection method can be a NULL operation.

TCP socket connections have been wrapped for in the connection
interface. This is the recommended connection type.

## Connection Manager and Message Handlers

The Message Manager and Handler (!MessageManager and !MessageHandler)
classes can be used to manage a connection that allows for multiple
message types to be handled. The message manager contains a list of
message handlers and executed the appropriate handler when a message is
received. These classes are particularly useful on robot controllers
which may have a limited number of connections available to them.

{ }}}}

## Wireshark Protocol Dissector

A Lua Wireshark dissector plugin for the simple message protocol is
available from
[ros-industrial/packet-simplemessage](https://github.com/ros-industrial/packet-simplemessage)
at !GitHub. See the readme for information on how to install it.

## Contact us/Technical support

For questions about this package or ROS-Industrial in general, please
contact the developers by posting a message in the
[ROS-Industrial category](https://discourse.ros.org/c/ros-industrial) on
ROS Discourse.

## Reporting bugs
