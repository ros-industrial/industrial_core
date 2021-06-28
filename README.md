# Industrial Core

[![Build Status: Ubuntu Bionic (Actions)](https://github.com/ros-industrial/industrial_core/workflows/CI%20-%20Ubuntu%20Bionic/badge.svg?branch=melodic-devel)](https://github.com/ros-industrial/industrial_core/actions?query=workflow%3A%22CI+-+Ubuntu+Bionic%22)
[![Build Status: Ubuntu Focal (Actions)](https://github.com/ros-industrial/industrial_core/workflows/CI%20-%20Ubuntu%20Focal/badge.svg?branch=melodic-devel)](https://github.com/ros-industrial/industrial_core/actions?query=workflow%3A%22CI+-+Ubuntu+Focal%22)
[![Github Issues](https://img.shields.io/github/issues/ros-industrial/industrial_core.svg)](http://github.com/ros-industrial/industrial_core/issues)

[![license - bsd 3 clause](https://img.shields.io/:license-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)


[ROS-Industrial][] core communications packages. See the [ROS wiki][] page for more information.


## Contents

Branch naming follows the ROS distribution they are compatible with. `-devel`
branches may be unstable. Releases are made from the distribution branches
(`hydro`, `indigo`, `jade`, `kinetic`, `lunar`, `melodic`).

Older releases may be found in the old ROS-Industrial [subversion repository][].


## Status

The packages in this repository are *community supported*.
This means they do not get support from an OEM, nor from the ROS-Industrial consortia directly (see also the `support level` badge at the top of this page).

Maintenance and development is on a best-effort basis and depends on volunteers.


## Installation

Binary packages are available for ROS Kinetic and ROS Melodic.

They can be installed using `apt` on Debian/Ubuntu.


### Example

To install `industrial_core` on Ubuntu Bionic for ROS Melodic (after having followed the normal ROS Melodic installation tutorial):

```
sudo apt install ros-melodic-industrial-core
```

This would install all the packages in this repository (and all their dependencies).


## Building

### On newer (or older) versions of ROS

Building the packages on newer (or older) versions of ROS is in most cases possible and supported. For example: building the packages in this repository on Ubuntu Focal/ROS Noetic systems is supported. This will require creating a Catkin workspace, cloning this repository, installing all required dependencies and finally building the workspace.

### Catkin tools

It is recommended to use [catkin_tools][] instead of the default [catkin][] when building ROS workspaces. `catkin_tools` provides a number of benefits over regular `catkin_make` and will be used in the instructions below. All packages can be built using `catkin_make` however: use `catkin_make` in place of `catkin build` where appropriate.

### Building the packages

The following instructions assume that a [Catkin workspace][] has been created at `$HOME/catkin_ws` and that the *source space* is at `$HOME/catkin_ws/src`. Update paths appropriately if they are different on the build machine.

These instructions build the `melodic-devel` branch on a ROS Melodic system:

```bash
# change to the root of the Catkin workspace
$ cd $HOME/catkin_ws

# retrieve the latest development version of industrial_core. If you'd rather
# use the latest released version, replace 'melodic-devel' with 'melodic'
$ git clone -b melodic-devel https://github.com/ros-industrial/industrial_core.git src/industrial_core

# check build dependencies. Note: this may install additional packages,
# depending on the software installed on the machine
$ rosdep update

# be sure to change 'melodic' to whichever ROS release you are using
$ rosdep install --from-paths src/ --ignore-src --rosdistro melodic

# build the workspace (using catkin_tools)
$ catkin build
```

### Activating the workspace

Finally, activate the workspace to get access to the packages just built:

```bash
$ source $HOME/catkin_ws/devel/setup.bash
```

At this point all packages should be usable (ie: `roslaunch` should be able to auto-complete package names starting with `industrial_..`). In case the workspace contains additional packages (ie: not from this repository), those should also still be available.


## ROS Distro Support

|         | Kinetic | Melodic |
|:-------:|:-------:|:-------:|
| Branch  | [`kinetic-devel`](https://github.com/ros-industrial/industrial_core/tree/kinetic-devel) |[`kinetic-devel`](https://github.com/ros-industrial/industrial_core/tree/kinetic-devel) |
| Status  | supported | supported |
| Version | [version](http://repositories.ros.org/status_page/ros_kinetic_default.html?q=industrial_core) | [version](http://repositories.ros.org/status_page/ros_melodic_default.html?q=industrial_core) |


[ROS-Industrial]: http://wiki.ros.org/Industrial
[ROS wiki]: http://wiki.ros.org/industrial_core
[subversion repository]: https://github.com/ros-industrial/swri-ros-pkg
[Catkin workspace]: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
[catkin]: http://wiki.ros.org/catkin
[catkin_tools]: https://catkin-tools.readthedocs.io/en/latest
