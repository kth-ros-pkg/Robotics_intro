^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_state_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2018-01-10)
------------------
* removed changelogs
* consistent package versions
* Contributors: Hilario Tome

0.3.2 (2018-01-12)
------------------

0.3.3 (2018-03-14)
------------------

0.3.4 (2018-03-28)
------------------

0.3.5 (2018-04-05)
------------------

0.3.6 (2018-05-02)
------------------

0.3.7 (2018-05-07)
------------------

0.3.8 (2018-05-08)
------------------

0.3.9 (2018-05-29)
------------------

0.3.10 (2018-06-15)
-------------------

0.3.11 (2018-06-19)
-------------------

0.13.1 (2017-11-06)
-------------------
* Update changelogs
* Contributors: Bence Magyar

0.13.0 (2017-08-10)
-------------------
* Update changelogs
* Contributors: Bence Magyar

0.12.3 (2017-04-23)
-------------------
* Update changelogs
* Contributors: Bence Magyar

0.12.2 (2017-04-21)
-------------------
* Update changelogs
* Contributors: Bence Magyar

0.12.1 (2017-03-08)
-------------------
* Update changelogs
* Contributors: Bence Magyar

0.12.0 (2017-02-15)
-------------------
* Update changelogs
* Merge pull request #242 from bmagyar/update_package_xmls
  Update package xmls
* Add Enrique and Bence to maintainers
* Merge branch 'kinetic-devel' into F_enable_part_traj_kinetic
* Contributors: Bence Magyar, beatrizleon

0.11.2 (2016-08-16)
-------------------
* Update changelogs
* Contributors: Bence Magyar

0.11.1 (2016-05-23)
-------------------
* Update changelogs
* Contributors: Bence Magyar

0.11.0 (2016-05-03)
-------------------
* Update changelogs
* Contributors: Bence Magyar

0.10.0 (2015-11-20)
-------------------
* Update changelogs
* Merge pull request #186 from ros-controls/w-unused-parameter
  Address -Wunused-parameter warnings
* Address -Wunused-parameter warnings
* Merge pull request #181 from ros-controls/jsc-extra-joints
  Joint state controller extra joints
* joint_state_controller: Add extra joints support
  Allow to optionally specify a set of extra joints for state publishing that
  are not contained in the JointStateInterface associated to the controller.
  The state of these joints can be specified via ROS parameters, and remains
  constant over time.
* joint_state_controller: Add test suite
* joint_state_controller: Migrate to package format2
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.2 (2015-05-04)
------------------
* Update changelogs.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.1 (2014-11-03)
------------------
* Update changelogs
* Merge pull request #152 from pal-robotics-forks/update-maintainers
  Update package maintainers
* Update package maintainers
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.0 (2014-10-31)
------------------
* Update changelogs
* Merge pull request #132 from ros-controls/fix_cmakelist_catkin
  Fixed incorrect ordering of catkin_package() call that breaks catkin 2.0
* Fixed incorrect ordering of catkin_package() call that breaks catkin 2.0 builds in CMakeLists.txt
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.8.1 (2014-07-11)
------------------
* Update chegelogs
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.8.0 (2014-05-12)
------------------
* Updated changelogs
* Merge pull request #92 from pal-robotics/install-missing-resources
  Add missing controller resources to install target
* Add missing controller resources to install target
* Merge pull request #91 from pal-robotics/indigo-devel
  Remove rosbuild artifacts. Fix #90.
* Remove rosbuild artifacts. Fix #90.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.7.2 (2014-04-01)
------------------
* Prepare 0.7.2
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.7.1 (2014-03-31)
------------------
* Prepare 0.7.1
* 0.7.0
* Prepare changelogs for 0.7.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.6.0 (2014-02-05)
------------------
* Updated changelogs
* Merge pull request #75 from pal-robotics/fix-linking
  Link controllers to catkin libraries
* Link shared libraries to catkin libraries
  GCC is quite lenient with missing symbols on shared libraries and
  doesn't event output any warning about it.
  When building with other compilers, missing symbols result in build
  errors.
* Merge pull request #66 from po1/install-default-config
  Install default config files
* Install default config files
* Merge branch 'hydro-devel' into development
* Merge branch 'hydro-devel' of https://github.com/willowgarage/ros_controllers into hydro-devel
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, Paul Mathieu

0.5.4 (2013-09-30)
------------------
* Updated changelogs
* Silence cppcheck warning on unit'ed variables.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.3 (2013-09-04)
------------------
* Update changelogs for 0.5.3.
* Merge branch 'hydro-devel' of https://github.com/willowgarage/ros_controllers into joint_trajectory_controller_hydro
* Removed last manifest.xml
* Merge pull request #37 from ros-controls/hydro_manifest_removed
  manifest.xml hidden in all packages
* Added ignored manifest.xml files, added rule to .gitignore
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.2 (2013-08-06)
------------------
* Updated changelogs
* Merge pull request #33 from ros-controls/effort_position_controller_fix
  Effort position controller fix
* Added joint limit enforcement for controller set point command
* Contributors: Dave Coleman

0.5.1 (2013-07-19)
------------------
* Merge pull request #32 from ros-controls/hydro-to-master-merge
  Merge hydro-devel to master
* Contributors: Dave Coleman

0.5.0 (2013-07-16)
------------------
* Merge pull request #31 from davetcoleman/hydro-master-merge
  Merged master branch into hydro-devel
* Merged master branch into hydro-devel
* Merged
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.
* Restore "Fixed PLUGINLIB_DECLARE_CLASS depreacated errors""
  This reverts commit 0862ad93696b0d736b565cd65ea36690dde0eaa7.
* Merge pull request #25 from jhu-lcsr-forks/hydro-devel
  Adding install targets for plugin xml files
* Adding install targets for plugin xml files
* Revert "Fixed PLUGINLIB_DECLARE_CLASS depreacated errors"
  This reverts commit 2314b8b434e35dc9c1c298140118a004e00febd8.
* Merge branch 'hardware_interface_rework' into sensor_interfaces
* Fix package URL in package.xml
* Merge branch 'master' into sensor_interfaces
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, Jonathan Bohren, wmeeusse

0.4.0 (2013-06-26)
------------------
* Version 0.4.0
* Merge pull request #23 from davetcoleman/master
  Removed PR2 references and renamed github repo in docs
* Removed PR2 references and renamed github repo in docs
* Merge pull request #18 from pal-robotics/master
  Fix package URLs in package.xml
* Fix package URL in package.xml
* Merge pull request #15 from davetcoleman/master
  Fixed PLUGINLIB_DECLARE_CLASS depreacated errors
* Fixed PLUGINLIB_DECLARE_CLASS depreacated errors
* Merge pull request #14 from pal-robotics/hardware_interface_rework
  Hardware interface rework
* Merge branch 'master' into hardware_interface_rework
* Propagate API changes in hardware_interface.
* Merge pull request #13 from jhu-lcsr-forks/master
  Adding cmake install targets
* adding install targets
* Merge pull request #8 from jhu-lcsr-forks/catkin
  Catkin
* adding switches for hybrid buildsystem
* adding back more manifests and makefiles
* merging, re-adding some makefiles and manifests
* Merge pull request #11 from pal-robotics/master
  Fix package URLs in manifest
* Fix package URL.
* bumping version
* adding package.xml files
* Catkinizing. Building, but could still be cleaned up
* use new root nodehandle to publish joint states in the namespace of the controller manager. This fixes a but when pushing the controller manager in a namespace, and keeps the same behavior when the controller manager is not in a namespace
* Merge pull request #6 from pal-robotics/master
  Minor maintenance fixes.
* Add .gitignore files on a per-package basis.
* Add missing include guard.
* Change tab indentation for spaces.
* port to new api with time and duration
* moved package with joint state controller
* Contributors: Adolfo Rodriguez Tsouroukdissian, Austin Hendrix, Dave Coleman, Jonathan Bohren, Wim Meeussen, hiDOF, wmeeusse
