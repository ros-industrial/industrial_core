## Overview

The `industrial_robot_client` package provides a standardized
interface for controlling industrial robots, based on the ROS-Industrial
[Specification](Industrial/Industrial_Robot_Driver_Spec). This package
includes a C++ reference implementation of the specification, using the
[simple_message](https://index.ros.org/p/simple_message) protocol to
communicate with a compatible server running on a standalone industrial
robot controller.

Primarily, this package provides the industrial_robot_client library.
The intent is for robot-specific implementations to reuse code from this
library using standard C++ derived-class mechanisms, to avoid much of
the copy/paste duplication in current industrial-robot driver
implementations. *Modification to adapt the library to new robot needs
is discussed in more detail
[below](#modification).*

This package also provides generic nodes exposing the base
industrial_robot_client functionality. If a robot does not require any
specific client-side code, then it may be sufficient to run these
standard nodes.
[This page](#generic_implementation)
describes this usage in more detail.

## Protocol documentation

The Simple Message (!SimpleMessage) protocol defines the message
structure between the ROS driver layer and the robot controller itself.

Refer to
[REP-I0006: Message Structures of the ROS-Industrial Simple Message Protocol](https://github.com/ros-industrial/rep/blob/master/rep-I0006.rst)
for a description of the protocol.

<a id="modification"></a>

## Modification for Robot-Specific Needs

Often, differences between manufacturer interfaces and robot designs
will require changes to the basic reference implementation provided in
the industrial_robot_client library. Joint-coupling, velocity scaling,
and communications protocols are all potential reasons for
robot-specific control code.

Rather than make a new copy of the entire ROS client codebase, it is
recommended that the integrator use a derived-class approach to only
re-implement the minimal functionality required. Wherever possible, the
new code should call the base class reference-implementation functions
to avoid code duplication and maintain consistent operations. It may be
helpful to review the library's
[design](#design) to determine
which functionality may need replacing. Simple joint reordering and
renaming can be handled through existing capabilities, as described
[here](#joint_naming).

### Example

The following example shows how to replace the reference velocity
calculation (0-100% of max-velocity) with an absolute velocity
algorithm.

1\. Create a new derived class, based on JointTrajectoryDownloader (or
JointTrajectoryStreamer) 

```cplusplus
class myRobot_JointTrajectoryDownloader : JointTrajectoryDownloader
{
...
}
```



1\. Override specific functionality. In this case, the velocity
calculation. 

```cplusplus
class myRobot_JointTrajectoryDownloader : JointTrajectoryDownloader
{

public:
  bool calc_velocity(const trajectory_msgs::JointTrajectoryPoint& pt, double* rbt_velocity)
  {
    if (pt.velocities.empty())
      *rbt_velocity = SAFE_SPEED;
    else
      *rbt_velocity = std::min(pt.velocities.begin(), pt.velocities.end());

    return true;
  }
}
```



1\. Implement a new node, using an instance of the appropriate
derived-class. 

```cplusplus
int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "motion_interface");

  myRobot_JointTrajectoryDownloader motionInterface;
  motionInterface.init();
  motionInterface.run();

  return 0;
}
```



### Further examples

Further examples of creating robot-specific interfaces may be found in
the following code samples:

- [ABB driver client](https://github.com/ros-industrial/abb/blob/kinetic-devel/abb_driver/src/abb_joint_downloader_node.cpp)

## Nodes

Besides the reusable libraries, this package also provides default
implementations of a set of generic nodes. The following is the ROS API
of those generic nodes.

### robot_state

Connects to a robot state server program on a controller and publishes
joint states.

#### Action Feedback

- `feedback_states`
  ([control_msgs/msg/FollowJointTrajectory](https://docs.ros.org/en/rolling/p/control_msgs/msg/FollowJointTrajectory.html)):
  Provide feedback of current vs. desired joint position (and
  velocity/acceleration).

#### Published Topics

- `joint_states`
  ([sensor_msgs/msg/JointState](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/JointState.html)):
  Joint state for all joints of the connected robot.
- `robot_status`
  ([industrial_msgs/msg/RobotStatus](https://docs.ros.org/en/rolling/p/industrial_msgs/msg/RobotStatus.html)):
  Aggregate controller/robot status (ie: drives enabled, motion
  possible, etc)

#### Parameters

- `robot_ip_address` (`str`, default: `none`): IP address of the
  controller to connect to. If this is not set the node will exit.
- `~port` (`integer`, default: `11002`): TCP port the robot state server
  program is listening on.
- `robot_description` (`str`, default: `none`): The urdf xml robot
  description.
- `controller_joint_names` (`[str, str, str, ..]`): (optional) A list
  containing all joints the driver should assume control over. If this
  parameter is not found, the driver will try to extract this from the
  `robot_description` parameter, but this will result in all joints in
  the urdf to be included. If the driver should only publish joint
  states for a subset of the joints in the urdf, the
  `controller_joint_names` parameter must be set.

### motion_download_interface

Connects to a motion relay server program on the controller and uploads
trajectories. Note: this is the *download* version of the interface,
which means the entire trajectory is uploaded to the motion relay server
program before motion is started.

#### Subscribed Topics

- `joint_states`
  ([sensor_msgs/msg/JointState](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/JointState.html)):
  Joint state for all joints of the connected robot.
- `joint_path_command`
  ([trajectory_msgs/msg/JointTrajectory](https://docs.ros.org/en/rolling/p/trajectory_msgs/msg/JointTrajectory.html)):
  Execute a pre-calculated joint trajectory on the robot.

#### Services

- `joint_path_command`
  ([industrial_msgs/srv/CmdJointTrajectory](https://docs.ros.org/en/rolling/p/industrial_msgs/srv/CmdJointTrajectory.html)):
  Execute a new motion trajectory on the robot.
- `stop_motion`
  ([industrial_msgs/srv/StopMotion](https://docs.ros.org/en/rolling/p/industrial_msgs/srv/StopMotion.html)):
  Stop execution of the current motion at the earliest time possible.

#### Parameters

- `robot_ip_address` (`str`, default: `none`): IP address of the
  controller to connect to. If this is not set the node will exit.
- `~port` (`integer`, default: `11000`): TCP port the motion relay
  server program is listening on.
- `robot_description` (`str`, default: `none`): The urdf xml robot
  description.
- `controller_joint_names` (`[str, str, str, ..]`, default: `none`):
  (optional) A list containing all joints the driver should assume
  control over. If this parameter is not found, the driver will try to
  extract this from the `robot_description` parameter, but this will
  result in all joints in the urdf to be included. If the driver should
  only control a subset of the joints in the urdf, the
  `controller_joint_names` parameter must be set.

### motion_streaming_interface

Connects to a motion relay server program on the controller and streams
trajectories. Note: this is the *streaming* version of the interface,
which means trajectories are relayed piecewise to the motion relay
server program and motion is expected to start immediately (or: as soon
as possible).

#### Subscribed Topics

- `joint_states`
  ([sensor_msgs/msg/JointState](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/JointState.html)):
  Joint state for all joints of the connected robot.
- `joint_path_command`
  ([trajectory_msgs/msg/JointTrajectory](https://docs.ros.org/en/rolling/p/trajectory_msgs/msg/JointTrajectory.html)):
  Execute a pre-calculated joint trajectory on the robot.

#### Services

- `joint_path_command`
  ([industrial_msgs/srv/CmdJointTrajectory](https://docs.ros.org/en/rolling/p/industrial_msgs/srv/CmdJointTrajectory.html)):
  Execute a new motion trajectory on the robot.
- `stop_motion`
  ([industrial_msgs/srv/StopMotion](https://docs.ros.org/en/rolling/p/industrial_msgs/srv/StopMotion.html)):
  Stop execution of the current motion at the earliest time possible.

#### Parameters

- `robot_ip_address` (`str`, default: `none`): IP address of the
  controller to connect to. If this is not set the node will exit.
- `~port` (`integer`, default: `11000`): TCP port the motion relay
  server program is listening on.
- `robot_description` (`str`, default: `none`): The urdf xml robot
  description.
- `controller_joint_names` (`[str, str, str, ..]`, default: `none`):
  (optional) A list containing all joints the driver should assume
  control over. If this parameter is not found, the driver will try to
  extract this from the `robot_description` parameter, but this will
  result in all joints in the urdf to be included. If the driver should
  only control a subset of the joints in the urdf, the
  `controller_joint_names` parameter must be set.

## Contact us/Technical support

For questions about this package or ROS-Industrial in general, please
contact the developers by posting a message in the
[ROS-Industrial category](https://discourse.ros.org/c/ros-industrial) on
ROS Discourse.

## Reporting bugs

## generic_implementation

### Overview

This page describes how to launch and interface to the generic
robot-interface nodes provided by the
industrial_robot_client package.
These nodes may be suitable to control "standard" robots that do not
require robot-specific client-side code. *For instructions on modifying
these nodes to accommodate robot-specific needs, please see the
industrial_robot_client page.*

### Usage

Before running the nodes, the user must first determine what
position-control method is required by the robot-side server code. The
ROS-Industrial [specification](Industrial/Industrial_Robot_Driver_Spec)
gives several alternative methods of position control, depending on the
application needs and robot capabilities. **NOTE: The ROS-Industrial
specification does allow for a low level streaming interface (sending
individual points to the robot, different than the streaming interface
below). This is not supported currently by the generic clients.**

All supported methods work with the `/joint_path_command` and
`/joint_states` topics, so should be compatible with standard ROS
`arm_navigation` and MoveIt stacks.

#### Streaming Interface

The *streaming* interface sends trajectory commands to the robot
on-the-fly. At the ROS level this is functionally equivalent to the
download interface, some robot require points be streamed at the driver
level: 

```
roslaunch industrial_robot_client robot_interface_streaming.launch robot_ip:=<value>
```



#### Download Interface

The *download* interface sends the entire trajectory to the robot before
executing any motion: 

```
roslaunch industrial_robot_client robot_interface_download.launch robot_ip:=<value>
```



#### Parameters

The generic robot-interface nodes look for key parameter values to
control their behavior. Some of these parameters are set in the launch
files. Others must be manually set by the end-user (or added to
robot-specific launch files):

- `robot_ip_address` \-- IP address of the robot controller
  \[required\]
- set in the launch files, based on the required robot_ip argument

<!-- -->

- `controller_joint_names` \-- list of joint names for ROS trajectory
  topics \[optional\]
- naming should match the robot's URDF file
- ordering should match the order required by the robot-controller
  server
- can be used for
  [joint re-ordering](#joint_naming)
- defaults to "joint_1, joint_2, \... joint_6" if not specified

<!-- -->

- `robot_description` \-- URDF (XML) description of this robot
  \[optional\]
- used to get maximum velocity limits for each joint, for velocity
  scaling
- must match the values in the robot controller for correct velocity
  scaling
- `JointTrajectoryInterface` defaults to a "safe" velocity if not
  specified

### Nodes

All `industrial_robot_client` generic launch files provide the same
nodes and topics:

#### robot_state

Provides feedback of robot position and status from the industrial robot
controller.

##### Published Topics

- `joint_states`
  ([control_msgs/msg/JointState](https://docs.ros.org/en/rolling/p/control_msgs/msg/JointState.html)):
  Joint trajectory feedback messages.
- `feedback_states`
  ([control_msgs/msg/FollowJointTrajectoryFeedback](https://docs.ros.org/en/rolling/p/control_msgs/msg/FollowJointTrajectoryFeedback.html)):
  Joint trajectory feedback messages.
- `robot_status`
  ([industrial_msgs/msg/RobotStatus](https://docs.ros.org/en/rolling/p/industrial_msgs/msg/RobotStatus.html)):
  Robot status messages.

##### Parameters

- `robot_ip_address` (`string`): IP address of robot-controller

#### motion_\*_interface

Sends joint trajectory messages to the industrial robot controller.

##### Subscribed Topics

- `joint_path_command`
  ([trajectory_msgs/msg/JointTrajectory](https://docs.ros.org/en/rolling/p/trajectory_msgs/msg/JointTrajectory.html)):
  Joint trajectory command messages.

##### Parameters

- `robot_ip_address` (`string`): IP address of robot-controller

#### joint_trajectory_action

Action interface for commanding robot motion.

##### Subscribed Topics

- `joint_trajectory_action/goal`
  ([control_msgs/msg/FollowJointTrajectoryActionGoal](https://docs.ros.org/en/rolling/p/control_msgs/msg/FollowJointTrajectoryActionGoal.html)):
  The goal describes the trajectory for the robot to follow.
- `joint_trajectory_action/cancel`
  ([actionlib_msgs/msg/GoalID](https://docs.ros.org/en/rolling/p/actionlib_msgs/msg/GoalID.html)):
  A request to cancel a specific goal.
- `feedback_states`
  ([control_msgs/msg/FollowJointTrajectoryFeedback](https://docs.ros.org/en/rolling/p/control_msgs/msg/FollowJointTrajectoryFeedback.html)):
  Listens to the state of the controller.

##### Published Topics

- `joint_path_command`
  ([trajectory_msgs/msg/JointTrajectory](https://docs.ros.org/en/rolling/p/trajectory_msgs/msg/JointTrajectory.html)):
  Sends joint trajectories to the controller.
- `joint_trajectory_action/feedback`
  ([control_msgs/msg/FollowJointTrajectoryActionFeedback](https://docs.ros.org/en/rolling/p/control_msgs/msg/FollowJointTrajectoryActionFeedback.html)):
  Feedback describing the progress the robot is making on following the
  trajectory.
- `joint_trajectory_action/status`
  ([actionlib_msgs/msg/GoalStatusArray](https://docs.ros.org/en/rolling/p/actionlib_msgs/msg/GoalStatusArray.html)):
  Provides status information on the goals that are sent to the action.
- `joint_trajectory_action/result`
  ([control_msgs/msg/FollowJointTrajectoryActionResult](https://docs.ros.org/en/rolling/p/control_msgs/msg/FollowJointTrajectoryActionResult.html)):
  Result of the commanded action

## design

The `industrial_robot_client` package contains several class/interface
implementations of basic ROS-Industrial functionality using the
[simple_message](https://index.ros.org/p/simple_message) protocol for
communication with industrial robot controllers. These classes can be
expanded through typical C++ derived-class functionality to implement
robot-specific functionality that differs from this reference
implementation (see
[here](#modification)).

This page gives an overview of the industrial_robot_client library
design, to help guide integrators in selecting which functionality to
override when implementing new robot interfaces.

### Overview

The industrial_robot_client library provides functionality for two main
nodes that should be present in every robot interface: 1.
**robot_state** node \-- publishes various topics related to the
robot's current position, status, etc. 1. **joint_trajectory** node \--
subscribes to the `command` topic, to issue joint-position commands to
the robot controller.

The functionality for these nodes is encapsulated into two primary
classes in the `industrial_robot_client` namespace:
`RobotStateInterface` and `JointTrajectoryInterface`. The node
executable programs contain minimal code: just an instantiation of the
class object and a few method-calls.

Integrators can derive new robot-specific classes from these base
classes and override individual method calls as needed. The following
sections describe these base classes and the main method calls in more
detail.

### State Publishing

The
[RobotStateInterface](http://ros.org/doc/groovy/api/industrial_robot_client/html/classindustrial__robot__client_1_1robot__state__interface_1_1RobotStateInterface.html)
class contains functionality for publishing various robot position and
status data at regular intervals, as received from the robot controller
connection.

This class is primarily a wrapper around the
[MessageManager](http://ros.org/doc/groovy/api/simple_message/html/classindustrial_1_1message__manager_1_1MessageManager.html)
class, which listens to the
[simple_message](https://index.ros.org/p/simple_message) robot
connection and processes each incoming message according to its *message
type*. In the default `RobotStateInterface` implementation, two
[MessageHandlers](http://ros.org/doc/groovy/api/simple_message/html/classindustrial_1_1message__handler_1_1MessageHandler.html)
are registered with the manager: 1.
[JointRelayHandler](http://ros.org/doc/groovy/api/industrial_robot_client/html/classindustrial__robot__client_1_1joint__relay__handler_1_1JointRelayHandler.html)
\-- publish current joint positions in the `joint_states` topic. 1.
[RobotStatusRelayHandler](http://ros.org/doc/groovy/api/industrial_robot_client/html/classindustrial__robot__client_1_1robot__status__relay__handler_1_1RobotStatusRelayHandler.html)
\-- publish current robot status info in the `robot_status` topic.

Messages are published at a rate determined by the robot controller
code. When new messages are received over the simple_message connection,
they are immediately published on the appropriate topics. There is no
buffering or polling, so messages are not guaranteed to be published at
regular intervals.

#### Key Methods

1\. `init()` \-- Establishes a connection to the robot controller.

- By default, this method establishes a simple_message TCP-socket
  connection to the address specified in the `robot_ip_address`
  parameter using a default port.
- Alternate forms use a robot connection specified by the calling code.
- This method also attaches the default message-handlers. 1. `run()`
  \-- Begin listening to messages from the robot controller. 1.
  `add_handler(\...)` \-- add (or replace) a new message handler

In general, integrators will probably not need to override any of the
`RobotStateInterface` methods. Instead, robot-specific functionality
may require altering the behavior of one of the default
message-handlers. To change message-handler behavior, derive a new class
from one of the existing handlers that implements the new behavior.
Then, call the add_handler method to replace the default handler of that
message type with the new robot-specific handler.

#### JointRelayHandler

The
[JointRelayHandler](http://ros.org/doc/groovy/api/industrial_robot_client/html/classindustrial__robot__client_1_1joint__relay__handler_1_1JointRelayHandler.html)
publishes the current robot joint positions. When a new message is
received, it is processed as follows: 

```
1) Retrieve all joint positions from the robot message
2) Call the create_message() method to generate ROS messages for publishing
  2.1) Call the transform() method to alter the reported positions, if needed
        - by default, this method does nothing
  2.2) Call the select() method to select a subset of joints for publishing
        - by default, this method implements the joint_name mapping functionality
3) Publish messages on the appropriate topics
4) Reply to the robot controller, if handshake requested
```



Integrators can override the `transform()` or `select()` methods to
implement robot-specific behavior, or can override the
`create_messages()` method to provide a completely custom
message-parsing sequence.

#### RobotStatusRelayHandler

TBD.

### Position Command

The
[JointTrajectoryInterface](http://ros.org/doc/groovy/api/industrial_robot_client/html/classindustrial__robot__client_1_1joint__trajectory__interface_1_1JointTrajectoryInterface.html)
class encapsulates most functionality for commanding robot joint
positions based on incoming ROS messages.

Since the ROS-Industrial
[specification](Industrial/Industrial_Robot_Driver_Spec) allows multiple
methods of transferring position commands to the robot controller, we
separate the message-transfer methods from the message-creation methods.
The `JointTrajectoryInterface` class is an abstract base-class,
providing methods for translating ROS messages into simple_message
messages. However, it only provides an *interface* specification for
transferring the messages to the robot controller. The
[JointTrajectoryDownloader](http://ros.org/doc/groovy/api/industrial_robot_client/html/classindustrial__robot__client_1_1joint__trajectory__downloader_1_1JointTrajectoryDownloader.html)
and
[JointTrajectoryStreamer](http://ros.org/doc/groovy/api/industrial_robot_client/html/classindustrial__robot__client_1_1joint__trajectory__streamer_1_1JointTrajectoryStreamer.html)
classes provide the required methods to transfer positions to the robot
controller, as described below.

#### Sequence of Operation

The `JointTrajectoryInterface` class subscribes to the `command`
topic, and listens to the incoming message stream. Each incoming
joint-position message is processed using the following steps:


```
1. Call trajectory_to_msgs() method to convert ROS message to simple_messages
      - Loop over all trajectory points, executing the following steps
      1.1 Call select() method to select/reorder joints for robot comms.
      1.2 Call transform() method to alter the reported positions, if needed
      1.3 Call calc_speed() method to calculate a scalar velocity representation
      1.4 Call create_message() method to create the simple_message
  2. Call send_to_robot() method to send trajectory to the robot controller
```



#### Key Methods

1\. `init()` \-- Establishes a connection to the robot controller.

- By default, this method establishes a simple_message TCP-socket
  connection to the address specified in the `robot_ip_address`
  parameter using a default port.
- Alternate forms use a robot connection specified by the calling code.
- This method also subscribes to the `command` topic 1. `run()` \--
  Begin listening to messages on the ROS topic. 1.
  `trajectory_to_msgs(\...)` \-- Convert ROS trajectory to a list of
  simple_messages 1. `select(\...)` \-- select/reorder points
- transforms from ROS ordering (arbitrary) to order expected by the
  robot server
- by default, this method implements the
  [joint_name](#joint_naming)
  mapping functionality 1. `transform(\...)` \-- alter the commanded
  joint position values
- by default, this method does nothing
- this can be overridden to implement robot-specific transforms (e.g.
  joint-coupling) 1. `calc_speed(\...)` \-- calculate a scalar
  velocity & duration
- the simple_message protocol expects a scalar velocity/duration, but
  the ROS topic provides a velocity for each joint 1.
  `calc_velocity(\...)` \-- calculate a scalar velocity from joint
  trajectory
- by default, velocity = the lowest joint_vel_ratio (joint_vel /
  joint_max_vel) 1. `calc_duration(\...)` \-- calculate a scalar
  duration from joint trajectory
- by default, duration = the expected time to complete the motion
- *NOTE: this uses the trajectory timestamps, not pos/velocity.* 1.
  `send_to_robot(\...)` \-- interface definition for a method to
  transfer command messages to the robot controller
- the `JointTrajectoryInterface` class does not provide an
  implementation of this method
- reference derived-class implementations are provided for both download
  and streaming methods, as described below.

In general, integrators should derive from one of the download/streaming
classes listed below and override the `select()`, `transform()`, or
`calc_velocity()` methods to implement robot-specific behavior. If a
non-standard robot connection is used, you could still derive from the
`JointTrajectoryInterface` class and provide a new robot-specific
`send_to_robot()` method, if appropriate.

#### Download/Streaming Classes

The
[JointTrajectoryDownloader](http://ros.org/doc/groovy/api/industrial_robot_client/html/classindustrial__robot__client_1_1joint__trajectory__downloader_1_1JointTrajectoryDownloader.html)
class is derived from the `JointTrajectoryInterface` base class, and
provides an implementation of the `send_to_robot()` method. This
implementation sends the entire trajectory to the robot as a sequence of
messages, with "magic" markers to signal the beginning and end of the
trajectory. The robot should begin executing the trajectory as soon as
the last trajectory point is received.

The
[JointTrajectoryStreamer](http://ros.org/doc/groovy/api/industrial_robot_client/html/classindustrial__robot__client_1_1joint__trajectory__streamer_1_1JointTrajectoryStreamer.html)
class is also derived from the `JointTrajectoryInterface` base class,
but provides a `send_to_robot()` implementation using a
position-streaming approach. In this method, a separate thread is used
to stream individual position commands to the robot controller. A new
position is not sent until the robot has executed the previous motion.
The robot-side implementation may use a small buffer of positions to
achieve smooth motion between successive commands.

### Generic Robot-Interface Nodes

The `industrial_robot_client` package also provides generic nodes that
run the basic reference implementation classes described above. These
nodes can be used for initial testing of new robots, or for robots that
do not require robot-specific processing. The following nodes are
provided:

- robot_state \-- runs RobotStateInterface reference implementation
- motion_streaming_interface \-- runs JointTrajectoryStreamer reference
  implementation
- motion_download_interface \-- runs JointTrajectoryDownloader reference
  implementation
- joint_trajectory_action \-- provides a basic actionlib interface

### File Summary

  -- ------------------------------------------------------------------ -- ----------------------------------------------------------- --
     <rowstyle="font-weight: bold; text-align: center;"> Filename      Description                                                 
     generic_joint_downloader_node.cpp                                     Main src-file for generic motion_download_interface node    
     generic_joint_streamer_node.cpp                                       Main src-file for generic motion_streaming_interface node   
     generic_robot_state_node.cpp                                          Main src-file for generic robot_state node                  
     joint_relay_handler.\*                                                message-handler for publishing joint positions              
     joint_trajectory_action.\*                                            actionlib interface                                         
     joint_trajectory_downloader.\*                                        position-command class using download method                
     joint_trajectory_interface.\*                                         position-command abstract base class                        
     joint_trajectory_streamer.\*                                          position-command class using streaming method               
     robot_state_interface.\*                                              state-publishing base-class                                 
     robot_status_relay_handler.\*                                         message-handler for publishing robot status                 
  -- ------------------------------------------------------------------ -- ----------------------------------------------------------- --

## joint_naming

This page describes the joint-remapping functionality provided in the
`industrial_robot_client` library, to support differences between ROS
and robot-controller joint ordering.

### Motivation

The ROS-Industrial
[specification](Industrial/Industrial_Robot_Driver_Spec) uses
[trajectory_msgs/msg/JointTrajectory](https://docs.ros.org/en/rolling/p/trajectory_msgs/msg/JointTrajectory.html)
and
[sensor_msgs/msg/JointState](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/JointState.html)
messages to communicate joint-position data with other ROS nodes. These
message types use an array to specify the ordering of joint data in each
individual message packet. No guarantee is made that the joint ordering
will be consistent between nodes or individual messages; only that a
single message is internally consistent.

By contrast, the simple_message protocol used for communication with the
robot controller expects the joints to appear in a pre-defined
(robot-specific) ordering.

The joint-remapping capability is used to re-order reported/commanded
joint-position data to accommodate differences between the ROS and
robot-controller representations. This capability is also flexible to
allow for adding or ignoring "dummy" joint values on either the ROS or
robot side.

### Usage

The joint-remapping functionality is controlled by providing a list of
joint-names to the industrial_robot_client node-base-class `init()`
methods. This list represents the joint names that should be associated
with the simple_message protocol communication to the robot controller.
The list of names should match the order (and count) of joint data
provided/expected by the robot controller's simple_message server code.

If no joint_names list is provided to the init() function, the node will
attempt to read a list from the `controller_joint_names` parameter, as
specified [here](https://index.ros.org/p/industrial_robot_client). If
this parameter doesn't exist, the node will assume a 6DOF robot, with
joint names `joint_1`, `joint_2`, \... `joint_6`.

*The provided list of joint names should match the names provided in the
robot's URDF file, to be compatible with other ROS navigation/planning
nodes. The ordering may be different and individual joints may be
added/omitted, depending on the robot-controller's simple_message
server requirements.*

### Examples

Once this joint-name list is provided, the nodes will automatically
adjust the published/received data to match. Specific examples are given
below:

#### Standard 6-DOF robot

If a standard 6-DOF robot's URDF uses standard joint names
(`joint_1`, \...) and the robot controller expects the joints in
base-to-tip order, no additional setup is required. The nodes will
automatically publish 6DOF joint positions under the standard names, and
will pass-through the commanded joint positions in joint order 1-6.

#### Joint Renaming

Some robots (e.g. Motoman) specify their joints using a non-standard
naming convention. Motoman joints are specified using letters
(`S,L,U,R,B,T`) rather than numbers (`1,2,3,4,5,6`). If the URDF
file is specified using a non-standard naming convention, then
joint-names should be specified to the industrial_robot_client nodes.

The easiest method is to create a robot-specific launch file that sets
the `controller_joint_names` parameter then calls the generic launch
file: 

```
<!-- motoman_streaming_interface.launch -->

<rosparam param="controller_joint_names">['joint_s','joint_l','joint_u','joint_r','joint_b','joint_t']</rosparam>
<include file="$(find industrial_robot_client)/launch/robot_interface_streaming.launch" />
```



#### Joint Reordering

This capability can also accommodate robot controllers that specify
joints in a non-standard order. For example, the motoman controller
specifies the SIA20D's 7th axis after the "standard" 6 axes, even
though it is physically placed in the middle of the arm. This is easily
handled by specifying the joint-name list as in the previous example,
using the order expected by the motoman controller:


```
<rosparam param="controller_joint_names">['joint_s','joint_l','joint_u','joint_r','joint_b','joint_t','joint_e']</rosparam>
```



Even though the robot URDF specifies the joints in a different order
(S,L,**E**,U,R,B,T), the joint-reordering is handled in the
`industrial_robot_client` nodes using the joint_names list.

#### Extra Joints

There may be "extra" joints specified by the robot controller that are
not defined in the robot's URDF file. These are handled by assigning
**blank names** in the joint_names list.

- when publishing joint_states, these "dummy" joints are ignored and
  not published
- when commanding new joint moves, a default (constant) value is used
  for the dummy joint
- *NOTE: this behavior may be unsafe, and may require a different
  default joint position, depending on the robot's configuration.*
  

```
rosparam set controller_joint_names "['joint_1','joint_2','','joint_3','joint_4','joint_5','joint_6']"
```



It is *not* correct to specify some other name (e.g. 'dummy') for the
extra joint. The `JointTrajectoryInterface` will not find the expected
'dummy' joint in the incoming ROS trajectories and will abort the
motion. All joints specified in the joint_names list **must** be
included in trajectory commands received from other ROS nodes.

#### Missing Joints

The robot controller may expect fewer joints than are specified in the
URDF file. This would be the case if multiple different robot
controllers are used to control a single ROS "robot". In this case,
the joint_names list specifies only the subset of joints expected by the
connected robot controller

- when publishing joint_states, all specified joints will be published
- *NOTE: This may cause issues with other ROS nodes, if all expected
  URDF joints are not present in the published state-stream*
- when commanding new joint positions, only the specified subset of
  joints will move. Other joint-commands will be ignored.



```
rosparam set controller_joint_names "['joint_1', 'joint_2', 'joint_3']"
```

## 

### Overview

The `industrial_robot_client` package provides a standardized
interface for controlling industrial robots, based on the ROS-Industrial
[Specification](Industrial/Industrial_Robot_Driver_Spec). This package
includes a C++ reference implementation of the specification, using the
[simple_message](https://index.ros.org/p/simple_message) protocol to
communicate with a compatible server running on a standalone industrial
robot controller.

Primarily, this package provides the industrial_robot_client library.
The intent is for robot-specific implementations to reuse code from this
library using standard C++ derived-class mechanisms, to avoid much of
the copy/paste duplication in current industrial-robot driver
implementations. *Modification to adapt the library to new robot needs
is discussed in more detail
[below](#modification).*

This package also provides generic nodes exposing the base
industrial_robot_client functionality. If a robot does not require any
specific client-side code, then it may be sufficient to run these
standard nodes.
[This page](https://index.ros.org/p/industrial_robot_client) describes
this usage in more detail.

### Protocol documentation

The Simple Message (!SimpleMessage) protocol defines the message
structure between the ROS driver layer and the robot controller itself.

Refer to
[REP-I0006: Message Structures of the ROS-Industrial Simple Message Protocol](https://github.com/ros-industrial/rep/blob/master/rep-I0006.rst)
for a description of the protocol.

<a id="modification"></a>

### Modification for Robot-Specific Needs

Often, differences between manufacturer interfaces and robot designs
will require changes to the basic reference implementation provided in
the industrial_robot_client library. Joint-coupling, velocity scaling,
and communications protocols are all potential reasons for
robot-specific control code.

Rather than make a new copy of the entire ROS client codebase, it is
recommended that the integrator use a derived-class approach to only
re-implement the minimal functionality required. Wherever possible, the
new code should call the base class reference-implementation functions
to avoid code duplication and maintain consistent operations. It may be
helpful to review the library's
[design](https://index.ros.org/p/industrial_robot_client) to determine
which functionality may need replacing. Simple joint reordering and
renaming can be handled through existing capabilities, as described
[here](https://index.ros.org/p/industrial_robot_client).

#### Example

The following example shows how to replace the reference velocity
calculation (0-100% of max-velocity) with an absolute velocity
algorithm.

1\. Create a new derived class, based on JointTrajectoryDownloader (or
JointTrajectoryStreamer) 

```cplusplus
class myRobot_JointTrajectoryDownloader : JointTrajectoryDownloader
{
...
}
```



1\. Override specific functionality. In this case, the velocity
calculation. 

```cplusplus
class myRobot_JointTrajectoryDownloader : JointTrajectoryDownloader
{

public:
  bool calc_velocity(const trajectory_msgs::JointTrajectoryPoint& pt, double* rbt_velocity)
  {
    if (pt.velocities.empty())
      *rbt_velocity = SAFE_SPEED;
    else
      *rbt_velocity = std::min(pt.velocities.begin(), pt.velocities.end());

    return true;
  }
}
```



1\. Implement a new node, using an instance of the appropriate
derived-class. 

```cplusplus
int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "motion_interface");

  myRobot_JointTrajectoryDownloader motionInterface;
  motionInterface.init();
  motionInterface.run();

  return 0;
}
```



#### Further examples

Further examples of creating robot-specific interfaces may be found in
the following code samples:

- [ABB driver client](https://github.com/ros-industrial/abb/blob/kinetic-devel/abb_driver/src/abb_joint_downloader_node.cpp)

### Nodes

Besides the reusable libraries, this package also provides default
implementations of a set of generic nodes. The following is the ROS API
of those generic nodes.

#### robot_state

Connects to a robot state server program on a controller and publishes
joint states.

##### Action Feedback

- `feedback_states`
  ([control_msgs/msg/FollowJointTrajectory](https://docs.ros.org/en/rolling/p/control_msgs/msg/FollowJointTrajectory.html)):
  Provide feedback of current vs. desired joint position (and
  velocity/acceleration).

##### Published Topics

- `joint_states`
  ([sensor_msgs/msg/JointState](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/JointState.html)):
  Joint state for all joints of the connected robot.
- `robot_status`
  ([industrial_msgs/msg/RobotStatus](https://docs.ros.org/en/rolling/p/industrial_msgs/msg/RobotStatus.html)):
  Aggregate controller/robot status (ie: drives enabled, motion
  possible, etc)

##### Parameters

- `robot_ip_address` (`str`, default: `none`): IP address of the
  controller to connect to. If this is not set the node will exit.
- `~port` (`integer`, default: `11002`): TCP port the robot state server
  program is listening on.
- `robot_description` (`str`, default: `none`): The urdf xml robot
  description.
- `controller_joint_names` (`[str, str, str, ..]`): (optional) A list
  containing all joints the driver should assume control over. If this
  parameter is not found, the driver will try to extract this from the
  `robot_description` parameter, but this will result in all joints in
  the urdf to be included. If the driver should only publish joint
  states for a subset of the joints in the urdf, the
  `controller_joint_names` parameter must be set.

#### motion_download_interface

Connects to a motion relay server program on the controller and uploads
trajectories. Note: this is the *download* version of the interface,
which means the entire trajectory is uploaded to the motion relay server
program before motion is started.

##### Subscribed Topics

- `joint_states`
  ([sensor_msgs/msg/JointState](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/JointState.html)):
  Joint state for all joints of the connected robot.
- `joint_path_command`
  ([trajectory_msgs/msg/JointTrajectory](https://docs.ros.org/en/rolling/p/trajectory_msgs/msg/JointTrajectory.html)):
  Execute a pre-calculated joint trajectory on the robot.

##### Services

- `joint_path_command`
  ([industrial_msgs/srv/CmdJointTrajectory](https://docs.ros.org/en/rolling/p/industrial_msgs/srv/CmdJointTrajectory.html)):
  Execute a new motion trajectory on the robot.
- `stop_motion`
  ([industrial_msgs/srv/StopMotion](https://docs.ros.org/en/rolling/p/industrial_msgs/srv/StopMotion.html)):
  Stop execution of the current motion at the earliest time possible.

##### Parameters

- `robot_ip_address` (`str`, default: `none`): IP address of the
  controller to connect to. If this is not set the node will exit.
- `~port` (`integer`, default: `11000`): TCP port the motion relay
  server program is listening on.
- `robot_description` (`str`, default: `none`): The urdf xml robot
  description.
- `controller_joint_names` (`[str, str, str, ..]`, default: `none`):
  (optional) A list containing all joints the driver should assume
  control over. If this parameter is not found, the driver will try to
  extract this from the `robot_description` parameter, but this will
  result in all joints in the urdf to be included. If the driver should
  only control a subset of the joints in the urdf, the
  `controller_joint_names` parameter must be set.

#### motion_streaming_interface

Connects to a motion relay server program on the controller and streams
trajectories. Note: this is the *streaming* version of the interface,
which means trajectories are relayed piecewise to the motion relay
server program and motion is expected to start immediately (or: as soon
as possible).

##### Subscribed Topics

- `joint_states`
  ([sensor_msgs/msg/JointState](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/JointState.html)):
  Joint state for all joints of the connected robot.
- `joint_path_command`
  ([trajectory_msgs/msg/JointTrajectory](https://docs.ros.org/en/rolling/p/trajectory_msgs/msg/JointTrajectory.html)):
  Execute a pre-calculated joint trajectory on the robot.

##### Services

- `joint_path_command`
  ([industrial_msgs/srv/CmdJointTrajectory](https://docs.ros.org/en/rolling/p/industrial_msgs/srv/CmdJointTrajectory.html)):
  Execute a new motion trajectory on the robot.
- `stop_motion`
  ([industrial_msgs/srv/StopMotion](https://docs.ros.org/en/rolling/p/industrial_msgs/srv/StopMotion.html)):
  Stop execution of the current motion at the earliest time possible.

##### Parameters

- `robot_ip_address` (`str`, default: `none`): IP address of the
  controller to connect to. If this is not set the node will exit.
- `~port` (`integer`, default: `11000`): TCP port the motion relay
  server program is listening on.
- `robot_description` (`str`, default: `none`): The urdf xml robot
  description.
- `controller_joint_names` (`[str, str, str, ..]`, default: `none`):
  (optional) A list containing all joints the driver should assume
  control over. If this parameter is not found, the driver will try to
  extract this from the `robot_description` parameter, but this will
  result in all joints in the urdf to be included. If the driver should
  only control a subset of the joints in the urdf, the
  `controller_joint_names` parameter must be set.

### Contact us/Technical support

For questions about this package or ROS-Industrial in general, please
contact the developers by posting a message in the
[ROS-Industrial category](https://discourse.ros.org/c/ros-industrial) on
ROS Discourse.

### Reporting bugs
