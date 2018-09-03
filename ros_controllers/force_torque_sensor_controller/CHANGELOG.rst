^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package force_torque_sensor_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Fix most catkin lint issues
* Sort dependencies
* Change for format2
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
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.2 (2015-05-04)
------------------
* Update changelogs.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.1 (2014-11-03)
------------------
* Update changelogs
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
* Merge pull request #66 from po1/install-default-config
  Install default config files
* Install default config files
* Merge branch 'hydro-devel' into development
* Merge branch 'hydro-devel' of https://github.com/willowgarage/ros_controllers into hydro-devel
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, Paul Mathieu

0.5.4 (2013-09-30)
------------------
* Updated changelogs
* Merge pull request #50 from ros-controls/plugin_xml_install
  Added install rules for plugin.xml
* Added install rules for plugin.xml
* Fix license header string for some files.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.3 (2013-09-04)
------------------
* Update changelogs for 0.5.3.
* Merge branch 'hydro-devel' of https://github.com/willowgarage/ros_controllers into joint_trajectory_controller_hydro
* Merge pull request #37 from ros-controls/hydro_manifest_removed
  manifest.xml hidden in all packages
* Removed manifest.xml from all packages to prevent rosdep heirarchy issues in Groovy and Hydro
* Added ignored manifest.xml files, added rule to .gitignore
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.2 (2013-08-06)
------------------
* Updated changelogs
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
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.
* Merge pull request #28 from pal-robotics/hardware_interface_sensors
  Controllers for exporting the state of force-torque and IMU sensors to ROS topics
* Make hybrid rosbuild-catkin packages.
  Affects force-torque and imu sensor controllers.
* Add package.xml scripts.
* Fix author name typo.
* Fix PLUGINLIB_DECLARE_CLASS depreacated errors.
* Propagate sensor interfaces API changes.
* Fix package URLs.
* Propagate changes in hardware_interface.
  - force-torque and IMU sensors no longer depend on Eigen.
  - The controllers that publish sensor state don't need the Eigen wrappers
  and now use the raw data directly.
* Controller publishing the state of a F/T sensor.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.4.0 (2013-06-26)
------------------
