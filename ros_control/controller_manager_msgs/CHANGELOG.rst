^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package controller_manager_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.6 (2018-01-08)
------------------
* deleted changelogs
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
* Merge pull request #204 from ros-controls/multi-iface-ctrl
  Allow controllers to claim resources from multiple interfaces
* Add helper to query rosparam controller configs
  There is no way to identify uninitialized controllers other than by inspecting
  the ROS parameter server, and looking for the required controller config
  structure, which is the existence of the <ctrl_name>/type parameter.
  This changeset adds Python helpers for performing this query.
* controller_manager_msgs: Multi-interface controllers
  - This is a ROS API and (internal) Python API breaking changeset.
  - Modify and extend ROS message definitions to allow for controllers that
  claim resources from more than one hardware interface.
  - Modify Python helpers and update the corresponding test suite to take into
  account above changes.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Bence Magyar, Hilario Tome, Sam Pfeiffer, Victor Lopez

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
* Contributors: Adolfo Rodriguez Tsouroukdissian

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
* Contributors: Dave Coleman

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
* Merge pull request #71 from davetcoleman/hydro-devel
  Renamed Github repos in docs, better error checking for spawning controllers
* Reneamed Github repo in documentation to ros-controls
* Merge branch 'fuerte_backport' into sensor_interfaces
* Merge branch 'master' into sensor_interfaces
* Merge branch 'master' into sensor_interfaces
* Merge branch 'master' into sensor_interfaces
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.4.0 (2013-06-25)
------------------
* Version 0.4.0
* 1.0.1
* Merge branch 'master' of github.com:willowgarage/ros_control
* Merge pull request #63 from pal-robotics/master
  Fix package URLs in package.xml
* Fix package URL in package.xml
* Merge branch 'master' into hardware_interface_rework
  Conflicts:
  hardware_interface/CMakeLists.txt
* Merge pull request #40 from jhu-lcsr-forks/catkin
  catkinizing, could still be cleaned up
* merging CMakeLists.txt files from rosbuild and catkin
* adding hybrid-buildsystem makefiles
* Merging from master, re-adding manifest.xml files
* Merge pull request #46 from pal-robotics/master
  Fix package URLs in manifest
* Fix package URLs.
* catkinizing, could still be cleaned up
* Adding in resource/claim infrastructure
* add state message
* clean up publishing controller state
* get rid of pr2 stuff
* Controller manager now runs with new ControllerLoader mechanism
* Merge branch 'fuerte'
* all pkgs now ported to fuerte
* running controller with casting. Pluginlib still messed up
* add macro
* compiling version
* working install target
* first catkin stuff
* Contributors: Adolfo Rodriguez Tsouroukdissian, Austin Hendrix, Dave Coleman, Jonathan Bohren, Vijay Pradeep, Wim Meeussen, wmeeusse
