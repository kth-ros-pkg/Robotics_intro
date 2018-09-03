^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package controller_manager_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.6 (2018-01-08)
------------------
* deleted changelogs
* fixed tests
* fixed missing return types and broken merge
* 0.2.5
* Updated changelog
* 0.2.4
* Updated changelog
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
* 0.11.4
* Update logs
* 0.11.3
* Update changelogs
* 0.11.2
* Update changelogs
* Merge pull request #254 from bmagyar/update_package_xmls
  Update package xmls
* Add Enrique and Bence to maintainer list
* 0.11.1
* Update changelog
* 0.11.0
* Update changelogs
* 0.10.1
* Update changelogs
* Add missing test dependency on rosservice
* Remove control_toolbox dependency. Fix thread linking error coming from removal of dependency.
* 0.10.0
* Update changelogs
* controller_manager_tests: Cleaner test exit
* Merge pull request #204 from ros-controls/multi-iface-ctrl
  Allow controllers to claim resources from multiple interfaces
* Add helper to query rosparam controller configs
  There is no way to identify uninitialized controllers other than by inspecting
  the ROS parameter server, and looking for the required controller config
  structure, which is the existence of the <ctrl_name>/type parameter.
  This changeset adds Python helpers for performing this query.
* controller_manager_tests: Extend test suite
  - Exercise much more of the controller_manager ROS API.
  - Create multi-interface test controllers and exercise them in tests.
* controller_manager: Multi-interface controllers
  - This is a C++ API breaking changeset.
  - Make controller_manager aware of controllers that claim resources from more
  than one hardware interface.
  - Update and extend the corresponding test suite.
* Merge pull request #205 from ros-controls/w-unused-parameter
  Address -Wunused-parameter warnings
* Address -Wunused-parameter warnings
* Contributors: Adolfo Rodriguez Tsouroukdissian, Bence Magyar, Hilario Tome, Hilario Tomé, Sam Pfeiffer, Victor Lopez

0.9.3 (2015-05-05)
------------------
* Update changelogs
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.2 (2015-05-04)
------------------
* Update changelogs
* Contributors: Adolfo Rodriguez Tsouroukdissian

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
* Merge pull request #165 from pal-robotics/cm-msgs-utils
  [controller_manager_msgs] Add Python helpers to make life easier.
* Add Python helpers to make life easier.
  - Helper for getting all active controller managers.
  - Helper for determining if a namespace contains the controller manager ROS API.
  - Helpers for filtering the output of the 'list_controllers' service by
  type, name, state, hardware_interface and claimed resources.
* Merge pull request #175 from bulwahn/indigo-devel
  make rostest in CMakeLists optional (ros/rosdistro#3010)
* make rostest in CMakeLists optional (ros/rosdistro#3010)
* Merge pull request #173 from shadowmanos/indigo-devel
  Fix spelling errors
* fix spelling errors
* Contributors: Adolfo Rodriguez Tsouroukdissian, Lukas Bulwahn, shadowmanos

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
* controller_manager_tests: fix library linking
  From patch provided by po1 on hydro-devel.
* Merge pull request #155 from pal-robotics/indigo-devel
  Remove rosbuild artifacts. Fix #154.
* Remove rosbuild artifacts. Fix #154.
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
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.6.0 (2014-02-05)
------------------
* Updated changelogs
* Contributors: Dave Coleman

0.5.8 (2013-10-11)
------------------
* "0.5.8"
* Updated changelogs
* Merge pull request #118 from ros-controls/no_manifest_xml
  Renamed manifest.xml to prevent conflicts with rosdep
* Renamed manifest.xml to prevent conflicts with rosdep
* Merge pull request #114 from vmayoral/hydro-devel
  CMakeLists fix to fit with OpenEmbedded/Yocto meta-ros layer.
* CMakeLists fix to fit with OpenEmbedded/Yocto meta-ros layer.
  Increase the compatibility of the ros_control code with
  meta-ros, an OpenEmbedded/Yocto layer that provides recipes for ROS
  packages disabling catking checking the variable CATKIN_ENABLE_TESTING.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, vmayoral

0.5.7 (2013-07-30)
------------------
* Updated changelogs
* Contributors: Dave Coleman

0.5.6 (2013-07-29)
------------------
* Updated changelogs
* Updated changelogs
* Contributors: Dave Coleman

0.5.5 (2013-07-23 17:04)
------------------------
* Updated changelogs
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
* Merge pull request #88 from ros-controls/master
  Merge master into hydro-devel for release to bloom
* Merge branch 'master' of github.com:ros-controls/ros_control into transmission_parsing
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.
* Merge branch 'master' of https://github.com/willowgarage/ros_control
* Merge pull request #81 from davetcoleman/master
  Pulled in changes in hydro-devel to master
* Merged hydro-devel into master
* Merge pull request #72 from jhu-lcsr-forks/hydro-devel
  adding install targets for plugin xml files
* adding install targets for plugin xml files
* Tests build.
* Fix build order.
* Merge branch 'fuerte_backport' into sensor_interfaces
* Revert "Fixed PLUGINLIB_DECLARE_CLASS deprecated errors"
  This reverts commit cd9aba265a380bafebb70d63081405d857e9380d.
* Merge branch 'master' into sensor_interfaces
* Contributors: Adolfo Rodriguez Tsouroukdissian, Austin Hendrix, Dave Coleman, Jonathan Bohren, wmeeusse

0.4.0 (2013-06-25)
------------------
* Version 0.4.0
* 1.0.1
* Merge pull request #56 from davetcoleman/master
  Deprecation Fixes, Documentation, and Spawner Namespace
* Fixed PLUGINLIB_DECLARE_CLASS deprecated errors
* Merge pull request #54 from pal-robotics/hardware_interface_rework
  Hardware interface rework
* Merge branch 'master' into hardware_interface_rework
  Conflicts:
  hardware_interface/CMakeLists.txt
* More uniform hardware_interface API. Refs  #45.
* Merge pull request #51 from jhu-lcsr-forks/master
  Adding cmake install targets
* adding install targets
* Merge pull request #40 from jhu-lcsr-forks/catkin
  catkinizing, could still be cleaned up
* adding missing manifests
* merging CMakeLists.txt files from rosbuild and catkin
* adding hybrid-buildsystem makefiles
* catkinizing, could still be cleaned up
* Merge branch 'master' of github.com:willowgarage/ros_control
* port to new time api
* add wait for service
* Adding in resource/claim infrastructure
* Refactoring joint command interfaces. Also added getJointNames()
* Switching to owned interfaces, instead of multiple virtual inheritance
* Changing interface names
* Getting tests compiling again
* Fixing copyright header text
* Joint interfaces now operate on pointers, instead of refs
* test for spawning mismatched interface fails correctly
* Basic spawn test works
* Spawning dummy controller works
* Merge branch 'fuerte'
* Tweaking inheritance to be virtual so it compiles. dummy app with controller manager compiles
* started controller_manager_tests. untested
* Contributors: Adolfo Rodriguez Tsouroukdissian, Austin Hendrix, Dave Coleman, Jonathan Bohren, Vijay Pradeep, Wim Meeussen, wmeeusse
