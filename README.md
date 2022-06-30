# industrial_core

[![license - bsd 3 clause](https://img.shields.io/:license-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)


## Quick start

`git clone` or download package to the `src` space of a Colcon workspace, install dependencies (using `rosdep`), build the workspace and activate the workspace (ie: `source` the appropriate `setup.bash` file, if using Bash).


## Status

The packages in this repository are *community supported*.
This means they do not get support from an OEM, nor from the ROS-Industrial consortia directly (see also the `support level` badge at the top of this page).

Maintenance and development is on a best-effort basis and depends on volunteers.


## Build and installation

The following instructions show an example workflow which would add `industrial_msgs` to the Colcon workspace at `$HOME/colcon_ws`, install the dependencies and build the workspace for ROS 2 Galactic.
If a different ROS 2 version should be used, or the Colcon workspace is located somewhere else, update commands where necessary.

**NOTE**: the `industrial_msgs` package is hosted in the `industrial_core` repository.
The `git clone` command will copy the files to a directory named `industrial_msgs`, to reflect the fact this is not a full `industrial_core` ROS 2 port, but only provides common interface definitions.

As follows:

```bash
# change to the root of the Colcon workspace
cd $HOME/colcon_ws

git clone \
  -b ros2_msgs_only \
  https://github.com/ros-industrial/industrial_core.git \
  src/industrial_msgs

# check build dependencies. First update the database.
# Note: this assumes ROS 2 Galactic. Change if/when necessary.
source /opt/ros/galactic/setup.bash
rosdep update --rosdistro=$ROS_DISTRO

# Note: this may install additional packages, depending on the software
# already installed on the machine
rosdep install \
  --from-paths src/industrial_msgs \
  --ignore-src \
  --rosdistro $ROS_DISTRO

# build the package (and only the package). To build the entire workspace,
# simply run 'colcon build'
colcon build --packages-upto industrial_msgs
```

If there were no warnings or errors, the workspace should now be activated using:

```bash
source $HOME/colcon_ws/install/local_setup.bash
```

At this point the interfaces provided by `industrial_msgs` should be available to ROS 2 applications and tools.
