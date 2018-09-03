^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.3 (2016-07-11)
------------------
* Changed set force to set angle in gazebo underactuated finger plugin
* Added gazebo world odometry that outputs rpy
* Contributors: Hilario Tome

1.1.8 (2018-05-17)
------------------
* Rotate object position using target link pose
* Contributors: Victor Lopez

1.1.7 (2018-03-29)
------------------
* Actively try to make the attachment on each world update loop
* Merge branch 'gazebo-attachment-plugin' into 'erbium-devel'
  Add gazebo_attachment plugin
  See merge request common/pal_gazebo_plugins!4
* Improve error checking of gazebo_attachment
* Add gazebo_attachment plugin
* Contributors: Hilario Tome, Victor Lopez

1.1.6 (2018-03-08)
------------------
* Merge branch 'titanium_simulation_issue' into 'erbium-devel'
  Control in effort when PID's are set. Otherwise control in position
  See merge request common/pal_gazebo_plugins!3
* Control in effort when PID's are set. Otherwise control in position
* Contributors: Adria Roig, Hilario Tome

1.1.5 (2018-01-30)
------------------
* added gazebo_ros depend
* Merge branch 'gazebo7' into erbium-devel
* Merge branch 'allow-params-on-namespace' into 'dubnium-devel'
  Allow gains to be pushed onto a namespace
  See merge request !2
* Allow gains to be pushed onto a namespace
* Added gazebo7 support
* Contributors: Hilario Tome, Hillario Tome, davidfernandez

1.1.4 (2016-10-14)
------------------
* Added missing depend
* Merge branch 'dubnium-devel' of gitlab:common/pal_gazebo_plugins into dubnium-devel
* Removed hardcoded base name in gazebo world odometry
* Changed world odom to use quaternion intstead of rpy
* 1.1.3
* Updated changelog
* Changed set force to set angle in gazebo underactuated finger plugin
* Added gazebo world odometry that outputs rpy
* Contributors: Hilario Tome

1.1.2 (2016-04-18)
------------------
* Merge branch 'finget_plugin_pid' into 'dubnium-devel'
  Finget plugin pid
  See merge request !1
* Being a bit more verbose on the initialization of the pluginç
* Cleanup
* Changed from set position to pid in finger plugin
* Remove wrongly placed link flag in GAZEBO_LIBRARIES
* Contributors: Hilario Tome, Sam Pfeiffer, Victor Lopez

1.1.1 (2016-04-15)
------------------
* Remove gazebo_ros_range, already merged into upstream gazebo_plugins
* Contributors: Victor Lopez

1.1.0 (2015-06-05)
------------------
* Remove Paul from maintainer
* Fix catkin_package dependency
* Add build and run depends on gazebo
* Add generic underactuated finger plugin for gazebo simulation
* Contributors: Luca Marchionni

1.0.1 (2014-11-17)
------------------
* Added plugin for harnessing the robot in simulation
* Adding plugin for wifi access point simulation in gazebo
* Simple plugin to move underactuated finger joints
* Deprecate PalModelPlugin
* Add launch files and run_gzserver script
* Catkinize, remove parts already in hydro
* Update to newer sdf API
* Move common code from robot-specific repos.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Enrique Fernandez, Luca Marchionni, Paul Mathieu
