^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simple_message
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.3 (2021-07-15)
------------------
* accept old defines as well for now, bw-compatibility for (`#262 <https://github.com/ros-industrial/industrial_core/issues/262>`_) (`#275 <https://github.com/ros-industrial/industrial_core/issues/275>`_)
* for a complete list of changes see the `commit log for 0.7.3 <https://github.com/ros-industrial/industrial_core/compare/0.7.2...0.7.3>`_.
* contributors: gavanderhoorn

0.7.2 (2021-06-28)
------------------
* target Melodic and newer.
* fix line-endings -- all files (`#268 <https://github.com/ros-industrial/industrial_core/issues/268>`_)
* add receive timeouts for simple socket (`#267 <https://github.com/ros-industrial/industrial_core/issues/267>`_)
* catkin_lint all packages (`#266 <https://github.com/ros-industrial/industrial_core/issues/266>`_)
* add Windows compatibility (`#264 <https://github.com/ros-industrial/industrial_core/issues/264>`_)
* improve TcpClient's connection re-establish behaviour (`#263 <https://github.com/ros-industrial/industrial_core/issues/263>`_)
* export compiler flags (`#262 <https://github.com/ros-industrial/industrial_core/issues/262>`_)
* remove exec permission on source files (`#259 <https://github.com/ros-industrial/industrial_core/issues/259>`_)
* add Noetic compatibility (`#258 <https://github.com/ros-industrial/industrial_core/issues/258>`_)
* update maintainers (`#243 <https://github.com/ros-industrial/industrial_core/issues/243>`_)
* for a complete list of changes see the `commit log for 0.7.2 <https://github.com/ros-industrial/industrial_core/compare/0.7.1...0.7.2>`_.
* contributors: Felix Messmer, Gaël Écorchard, Josh Langsfeld, Tim Perkins, ipa-nhg, Sean Yen, Paul Glaubitz, gavanderhoorn

0.7.1 (2019-09-19)
------------------
* Fix incorrect return value if deserializing duration fails on ``joint_traj_pt`` (`#226 <https://github.com/ros-industrial/industrial_core/issues/226>`_)
* Updated all package xml files to version 2 (`#232 <https://github.com/ros-industrial/industrial_core/issues/232>`_)
* all: update maintainer email addresses (`#222 <https://github.com/ros-industrial/industrial_core/issues/222>`_)
* Contributors: Gonzalo Casas, Jeremy Zoss, Jorge Nicho, gavanderhoorn

0.7.0 (2019-02-12)
------------------
* Updated the libraries install tags to follow the official documentation Fix `#193 <https://github.com/ros-industrial/industrial_core/issues/193>`_
* simple_message: get rid of obsolete gethostbyname. Fix `#197 <https://github.com/ros-industrial/industrial_core/issues/197>`_
* Added missed test dependencies to rosunit Fix `#205 <https://github.com/ros-industrial/industrial_core/issues/205>`_
* Fixed issue 157 regarding non-existing targets
* simple_message: build unit tests conditionally `#182 <https://github.com/ros-industrial/industrial_core/issues/182>`_
* Contributors: Alexander Rössler, Dmitry Rozhkov, Levi Armstrong, Nadia Hammoudeh García, Shaun Edwards

0.6.0 (2017-01-16)
------------------
* Added C++ 11 compile option
* Contributors: Victor Lamoine

0.5.1 (2017-01-15)
------------------
* Temporary fix, commented out disabled test to remove unstable build status
* Changed test port numbers to unused range in linux.  utest_message now uses port range defined by macros (addresses failure to init server socket)
* Amend to pull request `#153 <https://github.com/ros-industrial/industrial_core/issues/153>`_ (methods moved to TypedMessage)
* robot_status: missing reply to SERVICE_REQUEST. Fix in robot_status_message and relay_handler.
* simple_message: add doc to SpecialSeqValue enum members.
* simple_message: fix SpecialSeqValue typo. Fix `#130 <https://github.com/ros-industrial/industrial_core/issues/130>`_.
* Contributors: Alberto Marini, Shaun Edwards, gavanderhoorn

0.5.0 (2016-02-22)
------------------
* Switch ByteArray to <deque> for internal buffer
  - enables dynamic sizing, for larger messages
  * up to INT_MAX, which is unadvised
  - allows efficient data access at front/rear of msgs
  - maintains same external API
  * getRawDataPtr() is deprecated (not contiguous memory)
  - bugFix: SimpleSocket::receiveBytes() buffer-size check
  - update unit tests to handle ByteArray.MaxSize==INT_MAX
* Contributors: Jeremy Zoss

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
* Moved common socket contstructor code to simple_socket base class
* Updated simple message header to reflect vendor ranges specified in REP-I0004
* Correctly initialized connected state for udp connections
* Fixed issue `#48 <https://github.com/ros-industrial/industrial_core/issues/48>`_, logSocketError is now passed errno
* Merge pull request `#70 <https://github.com/ros-industrial/industrial_core/issues/70>`_ from gt-ros-pkg/hydro-devel
  Fixing receiveBytes for UDP
* Macro'ed out GETHOSTBYNAME, and fixed if-statement braces to be on a new line for consistency
* Added support for gethostbyname, for passing host names in addition to IP addresses.
* Making setConnected protected again, adding setDisconnected to public methods so that that method can be used to flag the connection as disconnected.
* Putting back in timeout for receiveBytes
* More formal fix for UDP communication.
  This should now make UDP sockets act almost exactly like the
  TCP sockets.
* Fixing receiveBytes for UDP
* robot_client: workaround for `#46 <https://github.com/ros-industrial/industrial_core/issues/46>`_. Fix `#67 <https://github.com/ros-industrial/industrial_core/issues/67>`_.
  This is an updated version of the workaround committed in 9df46977. Instead
  of requiring dependent packages to invoke the function defined in the
  CFG_EXTRAS cmake snippet, the snippet now sets up the linker path directly.
  Dependent packages now only need to remember to explicitly list their
  dependency on `industrial_robot_client` and `simple_message` in their
  `add_library(..)` statements.
* Contributors: Fred Proctor, Kelsey, Shaun Edwards, gavanderhoorn

0.3.4 (2014-01-21)
------------------
* robot_client: workaround for `#46 <https://github.com/ros-industrial/industrial_core/issues/46>`_. Fix `#67 <https://github.com/ros-industrial/industrial_core/issues/67>`_.
  This is an updated version of the workaround committed in 9df46977. Instead
  of requiring dependent packages to invoke the function defined in the
  CFG_EXTRAS cmake snippet, the snippet now sets up the linker path directly.
  Dependent packages now only need to remember to explicitly list their
  dependency on `industrial_robot_client` and `simple_message` in their
  `add_library(..)` statements.
* Contributors: gavanderhoorn

0.3.3 (2014-01-13)
------------------
* Fixed build issue due simple message library linking
* Contributors: gavanderhoorn

0.3.2 (2014-01-10)
------------------
* Removed header from industrial_utils/utils.h (not required)

0.3.1 (2014-01-09)
------------------
* Added polling check to socket read and muiltiple read calls in order to receive all desired bytes
* Removed library export from catkin macro.  Packages that depend on these must declare library dependencies explicitly (by name)
* Add error message to socket errors (instead of just errno).
* Converted to catkin
* Contributors: Christina Gomez, JeremyZoss, ROS, Shaun Edwards, gavanderhoorn, jrgnicho, kphawkins
