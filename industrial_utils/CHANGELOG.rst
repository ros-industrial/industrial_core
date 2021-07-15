^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package industrial_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.3 (2021-07-15)
------------------
* No changes

0.7.2 (2021-06-28)
------------------
* target Melodic and newer.
* fix line-endings -- all files (`#268 <https://github.com/ros-industrial/industrial_core/issues/268>`_)
* catkin_lint all packages (`#266 <https://github.com/ros-industrial/industrial_core/issues/266>`_)
* add Windows compatibility (`#264 <https://github.com/ros-industrial/industrial_core/issues/264>`_)
* add Noetic compatibility (`#258 <https://github.com/ros-industrial/industrial_core/issues/258>`_)
* update maintainers (`#243 <https://github.com/ros-industrial/industrial_core/issues/243>`_)
* for a complete list of changes see the `commit log for 0.7.2 <https://github.com/ros-industrial/industrial_core/compare/0.7.1...0.7.2>`_.
* contributors: Felix Messmer, ipa-nhg, Sean Yen, gavanderhoorn

0.7.1 (2019-09-19)
------------------
* Updated all package xml files to version 2 (`#232 <https://github.com/ros-industrial/industrial_core/issues/232>`_)
* all: update maintainer email addresses (`#222 <https://github.com/ros-industrial/industrial_core/issues/222>`_)
* Contributors: Jeremy Zoss, Jorge Nicho, gavanderhoorn

0.7.0 (2019-02-12)
------------------
* Added missed test dependencies to rosunit. Fix `#205 <https://github.com/ros-industrial/industrial_core/issues/205>`_
* Reworded error displayed when joints cannot be found Fix `#180 <https://github.com/ros-industrial/industrial_core/issues/180>`_
* Use urdf::*SharedPtr instead of boost::shared_ptr `#170 <https://github.com/ros-industrial/industrial_core/issues/170>`_
* Fixed issue with urdfdom headers
* Make building unit tests for client and utils conditional. Fix `#171 <https://github.com/ros-industrial/industrial_core/issues/171>`_.
* Switched to urdf::*SharedPtr instead of boost::shared_ptr to stay compatible.
* Contributors: Dmitry Rozhkov, G.A. vd. Hoorn, Jochen Sprickerhof, Levi Armstrong, Nadia Hammoudeh García, Shaun Edwards, ridhwanluthra

0.6.0 (2017-01-16)
------------------
* Added C++ 11 compile option
* Contributors: Victor Lamoine

0.5.1 (2017-01-15)
------------------
* No changes

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
* Fixed changelog links to point to main repo
* Contributors: Shaun Edwards

0.4.0 (2015-03-21)
------------------
* Silent warnings
* Contributors: Victor Lamoine

0.3.4 (2014-01-21)
------------------
* No change

0.3.3 (2014-01-13)
------------------
* No change

0.3.2 (2014-01-10)
------------------
* Removed header from industrial_utils/utils.h (not required)
* Contributors: Shaun Edwards

0.3.1 (2014-01-09)
------------------
* Remove obsolete export tags. Fix `#43 <https://github.com/ros-industrial/industrial_core/issues/43>`_.
* Fix `#52 <https://github.com/ros-industrial/industrial_core/issues/52>`_: ignore fixed joints in param_utils::getJointVelocityLimits
* Converted to catkin
* Contributors: JeremyZoss, Shaun Edwards, gavanderhoorn
