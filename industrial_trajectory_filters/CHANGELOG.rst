^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package industrial_trajectory_filters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.1 (2019-09-19)
------------------
* Updated all package xml files to version 2 (`#232 <https://github.com/ros-industrial/industrial_core/issues/232>`_)
* all: update maintainer email addresses (`#222 <https://github.com/ros-industrial/industrial_core/issues/222>`_)
* Contributors: Jeremy Zoss, Jorge Nicho, gavanderhoorn

0.7.0 (2019-02-12)
------------------
* Fix Melodic pluginlib incompatibility Fix `#204 <https://github.com/ros-industrial/industrial_core/issues/204>`_.
* Fixed compiler warning while including class_loader.h (change to class_loader.hpp)
* Updated filter base to return false when the update method returns false. Fix `#217 <https://github.com/ros-industrial/industrial_core/issues/217>`_.
* Contributors: Austin Deric, Levi Armstrong, Michael Ripperger

0.6.0 (2017-01-16)
------------------
* Added C++ 11 compile option
* Contributors: Victor Lamoine

0.5.1 (2017-01-15)
------------------
* Fix issue `#127 <https://github.com/ros-industrial/industrial_core/issues/127>`_
  Changed all relevant ROS_INFO_STREAM macros to ROS_DEBUG_STREAM macros. Now the filter only outputs the summary at the end.
* Contributors: Mart (mmj)

0.5.0 (2016-02-22)
------------------
* No changes

0.4.3 (2016-02-07)
------------------
* No changes

0.4.2 (2015-10-21)
------------------
* No change

0.4.1 (2015-03-23)
------------------
* No changes

0.4.0 (2015-03-21)
------------------
* Added a smoothing filter as a planning request adapter plugin
* Contributors: Chris Lewis

0.3.4 (2014-01-21)
------------------
* No change

0.3.3 (2014-01-13)
------------------
* No change

0.3.2 (2014-01-10)
------------------
* Removed header from industrial_utils/utils.h (not required)

0.3.1 (2014-01-09)
------------------
* Initial release
* Imported industrial trajectory filters package from industrial experimental
* Converted to catkin
* Contributors: Shaun Edwards, gavanderhoorn
