^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package controller_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.6 (2018-01-08)
------------------
* deleted changelogs
* 0.2.5
* Updated changelog
* Added ptr accesors
* 0.2.4
* Updated changelog
* Fixed bug hasTorqueSensor and hasAbsoluteSensor, added std run_time exception support to controller manager
* 0.2.3
* Updated changelog
* 0.2.2
* Update changelog
* 0.2.1
* Update changelog
* 0.2.0
* Update changelog
* 0.1.2
* Update changelogs
* 0.1.1
* Update changelogs
* 0.1.0
* Update changelogs
* Reset changelogs and version for pal release cycle
* Added better debug info for when a controller cannot be found
* Merge pull request #266 from ros-controls/kinetic-error-msg
  Fix misspelling revise message
* Fix misspelling revise message
* 0.11.4
* Update logs
* Merge pull request #261 from rojkov/kinetic-devel
  controller_manager: drop unused inclusion of tinyxml.h
* controller_manager: drop unused inclusion of tinyxml.h
  Signed-off-by: Dmitry Rozhkov <dmitry.rozhkov@linux.intel.com>
* 0.11.3
* Update changelogs
* 0.11.2
* Update changelogs
* Merge pull request #255 from bmagyar/use_vector_back
  to[to.size-1] to to.back()
* to[to.size-1] to to.back()
* Merge pull request #253 from bmagyar/cmake_warnings_boost
  Remove boost from depends declaration to fix cmake warning
* Remove boost from depends declaration to fix cmake warning
* Merge pull request #254 from bmagyar/update_package_xmls
  Update package xmls
* Add Enrique and Bence to maintainer list
* Clean up export leftovers from rosbuild
* Convert to format2, fix dependency in cmake
* 0.11.1
* Update changelog
* 0.11.0
* Update changelogs
* 0.10.1
* Update changelogs
* 0.10.0
* Update changelogs
* Merge pull request #210 from ros-controls/fix-do-switch-jade
  controller_manager: Fix doSwitch execution point - jade
* controller_manager: Fix doSwitch execution point
  The doSwitch method needs to be executed in the update() method,  that is, in
  the real-time path, which is where controller switching actually takes place.
  It was previously done in the switchController callback, which is non real-time.
  In this method controller switching is scheduled, but not actually executed.
  This changeset fixes a bug in which hardware interface  modes could switch
  before controllers, leading to undefined behavior.
* Merge pull request #218 from ros-controls/prepare-switch-jade
  Prepare switch jade
* Deprecate RobotHW::canSwitch
  Has been superceeded by RobotHW::prepareSwitch.
* Introduce prepareSwitch, replacement of canSwitch
  RobotHW::prepareSwitch is intended as a substitute for RobotHW::canSwitch.
  The main reasons for the change are a non-const signature to allow
  changing state and a more descriptive name.
  RobotHW::canSwitch will be deprecated in a later ROS distro.
* Merge pull request #204 from ros-controls/multi-iface-ctrl
  Allow controllers to claim resources from multiple interfaces
* controller_manager: Multi-interface controllers
  - This is a C++ API breaking changeset.
  - Make controller_manager aware of controllers that claim resources from more
  than one hardware interface.
  - Update and extend the corresponding test suite.
* Merge pull request #205 from ros-controls/w-unused-parameter
  Address -Wunused-parameter warnings
* Address -Wunused-parameter warnings
* Contributors: Adolfo Rodriguez Tsouroukdissian, Bence Magyar, Dave Coleman, Dmitry Rozhkov, Enrique Fernández Perdomo, Hilario Tome, Mathias Lüdtke, Sam Pfeiffer, Victor Lopez

0.9.3 (2015-05-05)
------------------
* Update changelogs
* controller_manager: Add missing rostest dep
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.2 (2015-05-04)
------------------
* Update changelogs
* Merge pull request #200 from ipa-mdl/strict_hwi_switch
  HW interface switch feature with unit tests
* added HW interface switch feature with unit tests
* Contributors: Adolfo Rodriguez Tsouroukdissian, Mathias Lüdtke

0.9.1 (2014-11-03)
------------------
* Update changelogs
* Merge pull request #191 from pal-robotics-forks/update-maintainers
  Update package maintainers
* Update package maintainers
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.0 (2014-10-31)
------------------
* Update changelogs
* Merge pull request #185 from pal-robotics/fwd-port-161
  Indigo port of #161
* spawner: changing language
* spawner: fixing shutdown message
* controller_manager: spawner: adding shutdown timeout to prevent deadlocks
* Merge pull request #173 from shadowmanos/indigo-devel
  Fix spelling errors
* fix spelling errors
* Contributors: Adolfo Rodriguez Tsouroukdissian, Jonathan Bohren, shadowmanos

0.8.2 (2014-06-25)
------------------
* Update changelogs
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.8.1 (2014-06-24)
------------------
* Update changelogs.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.8.0 (2014-05-12)
------------------
* Updated changelogs
* Merge pull request #155 from pal-robotics/indigo-devel
  Remove rosbuild artifacts. Fix #154.
* Remove rosbuild artifacts. Fix #154.
* Create README.md
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.7.2 (2014-04-01)
------------------
* Prepare 0.7.2
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.7.1 (2014-03-31)
------------------
* Prepare 0.7.1
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.7.0 (2014-03-28)
------------------
* Prepare 0.7
* Merge pull request #145 from pal-robotics/fix-spawner
  controller_manager: fix controller spawner
* controller_manager: remove global variable in spawner
  This minor change was mostly made to re-trigger the travis CI job
* Merge branch 'hydro-devel' of github.com:ros-controls/ros_control into hydro-devel
* controller_manager: fix controller spawner
  rosrun adds remapping arguments that conflict with argparse.
  This fixes the problem.
* Merge pull request #143 from pal-robotics/spawner-timeout
  Add a parameter to configure controller spawner timeout
* Add --timeout option to controller spawner
* Use argparse instead of getopt
  It is a much nicer interface
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, Paul Mathieu

0.6.0 (2014-02-05)
------------------
* Updated changelogs
* Update controller_manager.cpp
  Postfix to prefix increment operator.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.8 (2013-10-11)
------------------
* "0.5.8"
* Updated changelogs
* Merge pull request #118 from ros-controls/no_manifest_xml
  Renamed manifest.xml to prevent conflicts with rosdep
* Merge pull request #120 from ros-controls/extended_wait_time
  Extended wait time to 30 seconds for slower computers
* Fixed additional timeout that was just added
* Merge branch 'hydro-devel' into extended_wait_time
* Merge pull request #121 from pal-robotics/hydro-devel
  Fixes for next minor release
* Extended wait time to 30 seconds for slower computers
* Renamed manifest.xml to prevent conflicts with rosdep
* Fix broken unspawner script.
* Check controller_manager API early. Fast shutdown.
  - Check for all services required by spawner at the beginning, so it can know
  early on that it has all its requisites.
  - Remove service waiting from shutdown to ensure a fast teardown.
  Usecase: A spawner that dies after the controller manager should not wait
  for services to appear as they will never appear, the controllers are already
  stopped. This happens for example when killing a Gazebo session.
* Restore controller stop+unload on node kill.
  - Fixes #111.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.7 (2013-07-30)
------------------
* Updated changelogs
* Merge branch 'hydro-devel' of github.com:ros-controls/ros_control into hydro-devel
* Merge pull request #107 from kphawkins/hydro-devel
  Fix controller_manager.cpp reload-libraries/getControllerNames not clearing names first
* Update controller_manager.cpp
  getControllerNames now clears names before adding current names.  This fixes a bug in reloadControllerLibrariesSrv where the method is called twice in a row without first clearing the list.
  Steps to reproduce:
  - Spawn controller
  - Stop controller
  - reload-libraries
  controller_manager.cpp:501: bool controller_manager::ControllerManager::reloadControllerLibrariesSrv(controller_manager_msgs::ReloadControllerLibraries::Request&, controller_manager_msgs::ReloadControllerLibraries::Response&): Assertion `controllers.empty()' failed.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, kphawkins

0.5.6 (2013-07-29)
------------------
* Updated changelogs
* Updated changelogs
* Contributors: Dave Coleman

0.5.5 (2013-07-23 17:04)
------------------------
* Updated changelogs
* Tweaked Changelog
* Contributors: Dave Coleman

0.5.4 (2013-07-23 14:37)
------------------------
* Updated changelogs
* Contributors: Dave Coleman

0.5.3 (2013-07-22 18:06)
------------------------
* Updated changelog
* Contributors: Dave Coleman

0.5.2 (2013-07-22 15:00)
------------------------
* Updated CHANGELOGS
* Created changelogs for all packages
* Merge branch 'hydro-devel' of github.com:ros-controls/ros_control
* Contributors: Dave Coleman

0.5.1 (2013-07-19)
------------------
* Merge branch 'hydro-devel'
* Contributors: Dave Coleman

0.5.0 (2013-07-16)
------------------
* Merge branch 'hydro-devel' of github.com:ros-controls/ros_control into hydro-devel
* Merge pull request #88 from ros-controls/master
  Merge master into hydro-devel for release to bloom
* Removed urdf_interface dependencies
* Fix spawner choke when namespace is unspecified.
  Add missing check in conditional.
* Merge branch 'master' of github.com:ros-controls/ros_control into transmission_parsing
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.
* Merge branch 'master' of https://github.com/willowgarage/ros_control
* Merge pull request #81 from davetcoleman/master
  Pulled in changes in hydro-devel to master
* Merged hydro-devel into master
* Merge pull request #73 from jhu-lcsr-forks/hydro-devel
  Making script install target install scripts so that they are executable
* Making script install target install scripts so that they are executable
* Fix build order.
* Merge pull request #67 from davetcoleman/master
  Added user error checking to namespace argument
* Merge pull request #71 from davetcoleman/hydro-devel
  Renamed Github repos in docs, better error checking for spawning controllers
* Combined exceptions per jbohren
* Reneamed Github repo in documentation to ros-controls
* Merge branch 'fuerte_backport' into sensor_interfaces
* Better timeout error checking, necessary for Gazebo
* User error checking
* Merge branch 'master' of github.com:willowgarage/ros_control
* Merge branch 'master' into sensor_interfaces
* Merge branch 'master' into sensor_interfaces
* Merge branch 'master' into sensor_interfaces
* Contributors: Adolfo Rodriguez Tsouroukdissian, Austin Hendrix, Dave Coleman, Jonathan Bohren, wmeeusse

0.4.0 (2013-06-25)
------------------
* Version 0.4.0
* 1.0.1
* Merge pull request #56 from davetcoleman/master
  Deprecation Fixes, Documentation, and Spawner Namespace
* Merge pull request #65 from jhu-lcsr-forks/master
  Fixing failure mode in new catkin cmakelists
* Fixing failure mode in new catkin cmakelists
* Merge branch 'master' of github.com:willowgarage/ros_control
* Added namespace argument to spawner script
* Merge pull request #63 from pal-robotics/master
  Fix package URLs in package.xml
* Fix package URL in package.xml
* Merge branch 'master' of github.com:davetcoleman/ros_control
* Merge pull request #55 from ahendrix/master
  Minor catkinization fixes for python scripts.
* Python install for controller_manager.
* Fix build order dependency.
* Merge branch 'master' into hardware_interface_rework
  Conflicts:
  hardware_interface/CMakeLists.txt
* Merge pull request #51 from jhu-lcsr-forks/master
  Adding cmake install targets
* adding install targets
* Merge pull request #40 from jhu-lcsr-forks/catkin
  catkinizing, could still be cleaned up
* merging CMakeLists.txt files from rosbuild and catkin
* adding hybrid-buildsystem makefiles
* Merging from master, re-adding manifest.xml files
* Merge pull request #46 from pal-robotics/master
  Fix package URLs in manifest
* Fix package URLs.
* catkinizing, could still be cleaned up
* Merge pull request #37 from pal-robotics/master
  Issue #36 fix.
* Additional log feedback when load_controller fails
  When loading a controller fails bacause its configuration was not found on the
  parameter server, show the namespace where the parameters are expected to help
  debugging.
* Merge pull request #35 from pal-robotics/master
  Issue #33 fix.
* Remove unused method. Fixes #33.
* add option to pass in two nodehandles to a controller: one in the root of the controller manager namespace, and one in the namespace of the controller itself. This copies the behavior used by nodelets and nodes
* Merge pull request #30 from pal-robotics/master
  Documentation improvements
* Fix typo in rosdoc config files.
* Merge branch 'master' of github.com:willowgarage/ros_control into transmission_interface
* Merge pull request #26 from jbohren-forks/master
  Adding explicit header for recursive mutex
* Adding explicit header for recursive mutex
* Merge branch 'master' of github.com:willowgarage/ros_control into transmission_interface
* Merge pull request #24 from jbohren-forks/fix-controllers-rlock
  Alternative fix to getControllersByName mutex-locking requirements
* Removing getControllerByNameImpl
* Switching controller_manager controllers_lock\_ to be a recursive lock
* Merge branch 'master' of github.com:willowgarage/ros_control into transmission_interface
* Merge pull request #23 from jbohren-forks/inline-doc
  Adding lots of inline documentation, rosdoc files
* Fixing comment indent
* Adding template parameter doc
* Changing @ commands to \ commands
* More doc in controller manager
* Adding clearer ros warning in controller switching
* Adding lots of inline documentation, rosdoc files
  adding inline doc to robot_hw
  adding inline doc to robot_hw
  adding inline doc to robot_hw
  more doc
  more documentation
  more doc
  more doc
  more doc
  more doc
  formatting
  adding more doc groups in controller manager
  adding more doc groups in controller manager
  Adding doc for controllerspec
  adding hardware interface docs
  adding doc to joint interfaces
  adding rosdoc for controller_interface
  Adding / reformatting doc for controller interface
* Merge pull request #1 from jbohren-forks/fix-PID-unbounded-i_error
  Adding tests to show problems with integral term in ros_control pid_toolbox
* don't clear vectors in realtime
* Resolving conflict from new Pid API
* Merge branch 'master' into test-bad-integral-bounds
* Merge pull request #15 from pal-robotics/master
  Make public getControllerByName method thread-safe.
* Make public getControllerByName method thread-safe.
  Existing virtual non-threadsafe method has been suffixed with -Impl and pushed
  to protected class scope. In-class uses call getControllerByNameImpl, as the
  lock has already been acquired.
* Merge branch 'master' of github.com:willowgarage/ros_control
* new interface with time and duration
* add missing include
* remove .svn folder
* Doing resource conflict check on switchControllers call
* Adding in resource/claim infrastructure
* fix command line interface
* clean up publishing controller state
* Controller spec now also copies over type
* Switching to owned interfaces, instead of multiple virtual inheritance
* add scripts for controller manager
* get rid of pr2 stuff
* Controller manager can now register ControllerLoaders
* Controller manager now runs with new ControllerLoader mechanism
* Creating new plugin_loader interface
* Adding debugging printouts
* Namespacing controller_spec
* Fixing copyright header text
* Spawning dummy controller works
* Merge branch 'fuerte'
* Tweaking inheritance to be virtual so it compiles. dummy app with controller manager compiles
* all pkgs now ported to fuerte
* add missing file
* running controller with casting. Pluginlib still messed up
* add macro
* running version, with latest pluginlib
* compiling version
* compiling version
* first catkin stuff
* Contributors: Adolfo Rodriguez Tsouroukdissian, Austin Hendrix, Bob Holmberg, Dave Coleman, Jonathan Bohren, Vijay Pradeep, Wim Meeussen, hiDOF, wmeeusse
