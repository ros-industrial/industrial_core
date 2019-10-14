# Industrial Core

## ROS Distro Support

|         | Kinetic | Melodic |
|:-------:|:-------:|:-------:|
| Branch  | [`kinetic-devel`](https://github.com/ros-industrial/industrial_core/tree/kinetic-devel) |[`kinetic-devel`](https://github.com/ros-industrial/industrial_core/tree/kinetic-devel) |
| Status  | supported | supported |
| Version | [version](http://repositories.ros.org/status_page/ros_kinetic_default.html?q=industrial_core) | [version](http://repositories.ros.org/status_page/ros_melodic_default.html?q=industrial_core) |

## Travis - Continuous Integration

Status: [![Build Status](https://travis-ci.com/ros-industrial/industrial_core.svg?branch=kinetic-devel)](https://travis-ci.com/ros-industrial/industrial_core)

## ROS Buildfarm

|         | Kinetic Source | Kinetic Debian | Melodic Source | Melodic Debian |
|:-------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|
| industrial_core | [![released](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__industrial_core__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__industrial_core__ubuntu_xenial__source/) | [![released](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__industrial_core__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__industrial_core__ubuntu_xenial_amd64__binary/) | [![released](http://build.ros.org/buildStatus/icon?job=Msrc_uB__industrial_core__ubuntu_bionic__source)](http://build.ros.org/view/Msrc_uB/job/Msrc_uB__industrial_core__ubuntu_bionic__source/) | [![released](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__industrial_core__ubuntu_bionic_amd64__binary)](http://build.ros.org/view/Mbin_uB64/job/Mbin_uB64__industrial_core__ubuntu_bionic_amd64__binary/) |


[ROS-Industrial][] core meta-package. See the [ROS wiki][] page for more
information.

## License

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Contents

Branch naming follows the ROS distribution they are compatible with. `-devel`
branches may be unstable. Releases are made from the distribution branches
(`hydro`, `indigo`, `jade`).

Older releases may be found in the old ROS-Industrial [subversion repository][].


[ROS-Industrial]: http://wiki.ros.org/Industrial
[ROS wiki]: http://wiki.ros.org/industrial_core
[subversion repository]: https://github.com/ros-industrial/swri-ros-pkg

## Docker 

Industrial Core is also available as a Docker image from the [ROS-Industrial Docker Hub](https://hub.docker.com/u/rosindustrial).

Example usage:
```
docker run -it --rm rosindustrial/core:kinetic rosmsg show industrial_msgs/RobotStatus
```
