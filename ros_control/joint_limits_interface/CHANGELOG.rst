^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_limits_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.6 (2018-01-08)
------------------
* deleted changelogs
* Added urdfdom compatibility
* removed compatibility urdf header from joint limits interface
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
* -Werror=overloaded-virtual and initialization of fields in constructor
* joint_limits: fix test for open-loop velocity saturation
* joint_limits: use an open-loop policy for velocity staturation
  The feedback from the controller is way too slow to be used on an
  actual robot. A robot that had 15 rad.s^-2 on each wheel as
  an acceleration limit could not even reach 2 rad.s^-2
  This is in line with ros_controllers#23
* Merge pull request #221 from davetcoleman/jade-improve-joint-limits-error
  Throw error if EffortJointSaturationHandle is missing effort or velocity limits
* 0.11.4
* Update logs
* 0.11.3
* Update changelogs
* Merge pull request #259 from bmagyar/add_urdf_compatibility
  Add urdf compatibility header
* Add urdf compatibility header
* 0.11.2
* Update changelogs
* Merge pull request #254 from bmagyar/update_package_xmls
  Update package xmls
* Add Enrique and Bence to maintainer list
* Clean up export leftovers from rosbuild
* Convert to format2, fix dependency in cmake
* Merge pull request #251 from bmagyar/use_urdf_typedefs
  Replace boost::shared_ptr<urdf::XY> with urdf::XYConstSharedPtr
* Replace boost::shared_ptr<urdf::XY> with urdf::XYConstSharedPtr
* 0.11.1
* Update changelog
* 0.11.0
* Update changelogs
* 0.10.1
* Update changelogs
* Merge branch 'jade-devel' into jade-improve-joint-limits-error
* Merge pull request #229 from jspricke/fix_includes
  Don't export local include dirs
* Fix catkin_package
  * Don't export local include dirs.
  * Fix dependency on urdfdom (Thanks to Mathias Lüdtke).
* Throw error if EffortJointSaturationHandle is missing effort or velocity limits
* 0.10.0
* Update changelogs
* Contributors: Adolfo Rodriguez Tsouroukdissian, Adrià Roig, Bence Magyar, Daniel Pinyol, Dave Coleman, Hilario Tome, Jochen Sprickerhof, Paul Mathieu, Sam Pfeiffer, Victor Lopez

0.9.3 (2015-05-05)
------------------
* Update changelogs
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.2 (2015-05-04)
------------------
* Update changelogs
* Merge pull request #194 from ipa-mdl/limit_reset
  reset functionality for stateful position limit handles
* added tests for interface reset calls
* reset functionality for stateful postion handles
* Contributors: Adolfo Rodriguez Tsouroukdissian, Mathias Lüdtke

0.9.1 (2014-11-03)
------------------
* Update changelogs
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.0 (2014-10-31)
------------------
* Update changelogs
* Merge pull request #186 from pal-robotics/catkin-lint-fixes
  Buildsystem fixes suggested by catkin_lint
* Buildsystem fixes suggested by catkin_lint
* Merge pull request #183 from pal-robotics/joint-limits-interface-inline
  Add inline keyword to free header functions
* Add inline keyword to free header functions
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
* Merge pull request #172 from pal-robotics/liburdfdom-dev
  Propagate urdfdom changes to CMakeLists.txt
* Propagate urdfdom changes to CMakeLists.txt
  urdfdom is now standalone, so it must be find_package'd independently.
  Also, the rosparam rostest was not being built correctly.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.8.1 (2014-06-24)
------------------
* Update changelogs.
* Merge pull request #168 from pal-robotics/liburdfdom-dev
  Use upstream liburdfdom-dev package.
* Use upstream liburdfdom-dev package.
  Refs ros/rosdistro#4633.
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
* Merge pull request #149 from ros-controls/fix-devel-job
  Fix joint limits interface package dependencies.
* Fix package dependencies.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.7.0 (2014-03-28)
------------------
* Prepare 0.7
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.6.0 (2014-02-05)
------------------
* Updated changelogs
* Merge pull request #124 from jim-rothrock/hydro-devel
  Added the PositionJointSaturationInterface and VelocityJointSoftLimitsInterface classes.
* Updated the interface list.
* Added the PositionJointSaturationInterface and VelocitySoftLimitsInterface
  classes. There are now saturation and soft limit classes for effort-controlled,
  position-controlled, and velocity-controlled joints.
* Contributors: Dave Coleman, Jim Rothrock

0.5.8 (2013-10-11)
------------------
* "0.5.8"
* Updated changelogs
* Merge branch 'hydro-devel' into extended_wait_time
* Merge pull request #121 from pal-robotics/hydro-devel
  Fixes for next minor release
* Merge pull request #114 from vmayoral/hydro-devel
  CMakeLists fix to fit with OpenEmbedded/Yocto meta-ros layer.
* Merge pull request #116 from jim-rothrock/hydro-devel
  Added support for joints without soft limits.
* Added the EffortJointSaturationHandle and EffortJointSaturationInterface
  classes. They are used with joints that do not have soft limits specified in
  their URDF files.
* Minor documentation precision.
* Make position joint limits handle opn loop.
  - Lowers the entry barrier for simple robots without velocity measurements,
  poor control tracking or with a slow update rate.
* Update README.md
* Create README.md
* CMakeLists fix to fit with OpenEmbedded/Yocto meta-ros layer.
  Increase the compatibility of the ros_control code with
  meta-ros, an OpenEmbedded/Yocto layer that provides recipes for ROS
  packages disabling catking checking the variable CATKIN_ENABLE_TESTING.
* Fix license header in some files.
* Merge pull request #108 from ros-controls/ignore_joint_limits_manifest
  Ignore joint_limits_interfest manifest.xml
* Renamed joint_limits_interface manifext.xml
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, Jim Rothrock, vmayoral

0.5.7 (2013-07-30)
------------------
* Updated changelogs
* Contributors: Dave Coleman

0.5.6 (2013-07-29)
------------------
* Updated changelogs
* Merge branch 'hydro-devel' of github.com:ros-controls/ros_control into hydro-devel
* Updated changelogs
* Merge pull request #104 from pal-robotics/hydro-devel
  Add angle_wraparound joint limit property.
* Add angle_wraparound joint limit property.
  For full compatibility with MoveIt!'s joint limit specification.
  Note that we still have the extra effort and jerk specification.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

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
* Fixed gtests for joint_limits_interface in catkin
* Merge pull request #93 from pal-robotics/master
  joint_limits_interface broken in Groocy and Hydro
* Fix for joint_limits tests in catkin
* Restore urdf dependencies.
  Add conditional compilation for Fuerte and Groovy+ distros.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.1 (2013-07-19)
------------------
* Merge branch 'hydro-devel'
* Contributors: Dave Coleman

0.5.0 (2013-07-16)
------------------
* Made joint_limits_interface match hydro version number
* Merge pull request #88 from ros-controls/master
  Merge master into hydro-devel for release to bloom
* Removed urdf_interface dependencies
* Merge branch 'master' of github.com:ros-controls/ros_control into transmission_parsing
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.
* Better documentation of YAML joint limits spec.
  - Add cross-references in doc main page.
* Merge branch 'master' of https://github.com/willowgarage/ros_control
* Documentation improvements.
  - More consistency between transmission and joint limits interfaces doc.
  - Make explicit that these interfaces are not meant to be used by controllers,
  but by the robot abstraction.
* Merge pull request #82 from isanchez12/master
  Adding missing build dependency rostest.
* build dependency rostest added to package.xml and rostest added to CMakeLists.txt
* Added dependency for rostest to fix build error
* Fix compiler warnings (-Wreorder)
* Minor doc structure improvements.
* Add main page to joint_limits_interface doc.
* Merge pull request #76 from pal-robotics/joint_limits_interface
  Joint limits interface
* Remove temporary file from version control.
* Add attribution for soft_limits code.
  - Soft-limits enforcing is based on a previous implementation by Willow Garage.
  Add them in the copyright holders list.
* Lower severity of log message.
* Allow unsetting limits specification from rosparam.
  - Update tests.
* Add .gitignore
* Add joint limits parsing from rosparam + unit test.
* Add max_jerk to limits specification.
* Minor maintenance fixes.
* Add documentation.
* Extensive file, namespace, class renaming.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, Ish Sanchez, Jonathan Bohren

0.4.0 (2013-06-25)
------------------
