^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_ros_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.7 (2018-01-12)
------------------
* fixed unitialized value
* Contributors: Hilario Tome

2.6.6 (2018-01-12)
------------------

2.6.5 (2018-01-11)
------------------
* removed changelogs and unified package versions
* added updating of joint mode
* added joint mode
* updated gazebo ros control to support joint mode
* Fix gazebo8 warnings part 7: ifdef's for Joint::GetAngle and some cleanup (#642)
  * fix major version check >= 8, instead of > 8
  * gazebo_ros_bumper: use new API in commented code
  * gazebo_ros_api_plugin: world pose in local vars
  * worldLinearVel as local var in hand of god plugin
  * gazebo8+: Joint::GetAngle -> Joint::Position
* Contributors: Hilario Tome, Steven Peters

2.5.14 (2017-12-11)
-------------------
* Generate changelogs
* Replace Events::Disconnect* with pointer reset (#623)
* Contributors: Jose Luis Rivero, Steven Peters

2.5.13 (2017-06-24)
-------------------
* Update changelogs
* Less exciting console output (#561)
* Add catkin package(s) to provide the default version of Gazebo - take II (kinetic-devel) (#571)
  * Added catkin package gazebo_dev which provides the cmake config of the installed Gazebo version
  Conflicts:
  gazebo_plugins/package.xml
  gazebo_ros/package.xml
  gazebo_ros_control/package.xml
  * gazebo_plugins/gazebo_ros: removed dependency SDF from CMakeLists.txt
  The sdformat library is an indirect dependency of Gazebo and does not need to be linked explicitly.
  * gazebo_dev: added execution dependency gazebo
* Contributors: Dave Coleman, Jose Luis Rivero

2.5.12 (2017-04-25)
-------------------
* Changelogs for next version
* Fixed broken gazebo_ros_control tutorial link (#566)
* Contributors: Ian McMahon, Jose Luis Rivero

2.5.11 (2017-04-18)
-------------------
* Changelogs to prepare for next 2.5.11
* Change build system to set DEPEND on Gazebo/SDFormat (fix catkin warning)
  Added missing DEPEND clauses to catkin_package to fix gazebo catkin warning. Note that after the change problems could appear related to -lpthreads errors. This is an known issue related to catkin: https://github.com/ros/catkin/issues/856.
* Make gazebo_ros_control compatible with ros_control with respect to <hardwareInterface> tag (#550)
  * ros_control expects "<hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>", i.e. "hardware_interface/" prefix
  * add deprecation warning
  * improve warning
  * fix warning message fix
* Contributors: Andreas Bihlmaier, Dave Coleman, Jose Luis Rivero

2.5.10 (2017-03-03)
-------------------
* Changelogs for 2.5.10
* Revert catkin warnings to fix regressions (problems with catkin -lpthreads errors)
  For reference and reasons, please check:
  https://discourse.ros.org/t/need-to-sync-new-release-of-rqt-topic-indigo-jade-kinetic/1410/4
  * Revert "Fix gazebo catkin warning, cleanup CMakeLists (#537)"
  This reverts commit 5a0305fcb97864b66bc2e587fc0564435b4f2034.
  * Revert "Fix gazebo and sdformat catkin warnings"
  This reverts commit 11f95d25dcd32faccd2401d45c722f7794c7542c.
* Contributors: Jose Luis Rivero

2.5.9 (2017-02-20)
------------------
* Update changelogs
* Fix gazebo catkin warning, cleanup CMakeLists (#537)
* Namespace console output (#543)
  Namespace all console output
* Merge pull request #541 from bmagyar/print-jnt-name-of-unknown-hw-interface
  Print name of joint with wrong interface
* Print name of joint with wrong interface
* Merge pull request #539 from davetcoleman/kinetic-whitespace
  Removed all trailing whitespace
* Removed all trailing whitespace
* Merge pull request #519 from jspricke/urdfdom_headers_fix_kinetic
  Change boost::shared_ptr to urdf::JointConstSharedPtr
* Change boost::shared_ptr to urdf::JointConstSharedPtr
* Contributors: Bence Magyar, Dave Coleman, Jochen Sprickerhof, Jose Luis Rivero

2.5.8 (2016-12-06)
------------------
* Update changelogs for 2.5.8
* Contributors: Jose Luis Rivero

2.5.7 (2016-06-10)
------------------
* Update changelogs
* Update gazebo_ros_control version in package.xml to be able to run bloom for new release
* delete CATKIN_IGNORE in gazebo_ros_control (#456)
* Contributors: Jackie Kay, Jose Luis Rivero

2.5.6 (2016-04-28)
------------------

2.5.4 (2016-04-27)
------------------
* Merge pull request #454 from scpeters/merge_ijk
  merge indigo, jade to kinetic-devel
* merge indigo, jade to kinetic-devel
* Merge branch 'kinetic-devel' of https://github.com/ros-simulation/gazebo_ros_pkgs into kinetic-devel
* Upgrade to gazebo 7 and remove deprecated driver_base dependency (#426)
  * Upgrade to gazebo 7 and remove deprecated driver_base dependency
  * disable gazebo_ros_control until dependencies are met
  * Remove stray backslash
* Merge pull request #430 from ros-simulation/kinetic-devel-maintainer
  Update maintainer for Kinetic release
* disable gazebo_ros_control until dependencies are met
* Update also the gazebo_ros_control package
* disable gazebo_ros_control until dependencies are met
* Contributors: Hugo Boyer, Jackie Kay, Jose Luis Rivero, Steven Peters, William Woodall

2.5.3 (2016-04-11)
------------------
* Update changelogs for 2.5.3
* Merge branch 'jade-devel' into issue_387_remove_ros_remappings
* Contributors: Jose Luis Rivero, Martin Pecka

2.5.2 (2016-02-25)
------------------
* Prepare changelogs
* clean up merge from indigo-devel
* merging from indigo-devel
* Use Joint::SetParam for joint velocity motors
  Before gazebo5, Joint::SetVelocity and SetMaxForce
  were used to set joint velocity motors.
  The API has changed in gazebo5, to use Joint::SetParam
  instead.
  The functionality is still available through the SetParam API.
  cherry-picked from indigo-devel
  Add ifdefs to fix build with gazebo2
  It was broken by #315.
  Fixes #321.
* 2.4.9
* Generate changelog
* Merge pull request #350 from ros-simulation/indigo-devel_merged_from_jade
  Merge changes from jade-devel into indigo-devel
* Import changes from jade-branch
* Merge pull request #343 from ipa-fxm/gazebo_ros_control_review_dependencies
  [gazebo_ros_control] add missing dependencies
* add missing dependencies
* Merge pull request #332 from akio/fix-ros-control-param-ns
  gazebo_ros_control: Fix DefaultRobotHWSim puts robotNamespace twice
* Fix DefaultRobotHWSim puts robotNamespace twice
  DefaultRobotHWSim::initSim() member function uses both
  namespaced NodeHandle and robot_namespace string to create
  parameter names.
  For example,  if a robotNamespace is "rrbot",
  DefaultRobotHWSim tries to get parameters from following names:
  - /rrbot/rrbot/gazebo_ros_control/pid_gains/*
  - /rrbot/rrbot/joint_limits/*
  This commit change these names to:
  - /rrbot/gazebo_ros_control/pid_gains/*
  - /rrbot/joint_limits/*
* Merge pull request #2 from ros-simulation/indigo-devel
  Indigo devel
* Merge pull request #322 from ros-simulation/issue_321
  Add ifdefs to fix build with gazebo2
* Add ifdefs to fix build with gazebo2
  It was broken by #315.
  Fixes #321.
* Merge pull request #315 from ros-simulation/max_force
  Use Joint::SetParam for joint velocity motors
* Merge pull request #314 from ros-simulation/gazebo_cpp11
  Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* Use Joint::SetParam for joint velocity motors
  Before gazebo5, Joint::SetVelocity and SetMaxForce
  were used to set joint velocity motors.
  The API has changed in gazebo5, to use Joint::SetParam
  instead.
  The functionality is still available through the SetParam API.
* Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* Contributors: Adolfo Rodriguez Tsouroukdissian, Akiyoshi Ochiai, John Hsu, Jose Luis Rivero, Steven Peters, iche033, ipa-fxm

2.5.1 (2015-08-16 02:31)
------------------------
* Generate changelogs
* Merge pull request #339 from ros-simulation/fix-ros-control-param-ns-jade
  [jade] Fix DefaultRobotHWSim puts robotNamespace twice
* Fix DefaultRobotHWSim puts robotNamespace twice
  DefaultRobotHWSim::initSim() member function uses both
  namespaced NodeHandle and robot_namespace string to create
  parameter names.
  For example,  if a robotNamespace is "rrbot",
  DefaultRobotHWSim tries to get parameters from following names:
  - /rrbot/rrbot/gazebo_ros_control/pid_gains/*
  - /rrbot/rrbot/joint_limits/*
  This commit change these names to:
  - /rrbot/gazebo_ros_control/pid_gains/*
  - /rrbot/joint_limits/*
* Merge pull request #330 from ros-simulation/issue_323
  run_depend on libgazebo5-dev (#323)
* Added a comment about the need of libgazebo5-dev in runtime
* Added elevator plugin
* Merge pull request #336 from ros-simulation/jade-devel-c++11
  Use c++11
* Use c++11
* run_depend on libgazebo5-dev (#323)
  Declare the dependency.
  It can be fixed later if we don't want it.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Akiyoshi Ochiai, Jose Luis Rivero, Nate Koenig, Steven Peters

2.5.0 (2015-04-30)
------------------
* changelogs
* run_depend on libgazebo5-dev instead of gazebo5
* changelogs
* [style] remove trailing whitespace
* change the rosdep key for gazebo to gazebo5
* Contributors: Steven Peters, William Woodall

2.4.9 (2015-08-16 01:30)
------------------------
* Generate changelog
* Merge pull request #350 from ros-simulation/indigo-devel_merged_from_jade
  Merge changes from jade-devel into indigo-devel
* Import changes from jade-branch
* Merge pull request #343 from ipa-fxm/gazebo_ros_control_review_dependencies
  [gazebo_ros_control] add missing dependencies
* add missing dependencies
* Merge pull request #332 from akio/fix-ros-control-param-ns
  gazebo_ros_control: Fix DefaultRobotHWSim puts robotNamespace twice
* Fix DefaultRobotHWSim puts robotNamespace twice
  DefaultRobotHWSim::initSim() member function uses both
  namespaced NodeHandle and robot_namespace string to create
  parameter names.
  For example,  if a robotNamespace is "rrbot",
  DefaultRobotHWSim tries to get parameters from following names:
  - /rrbot/rrbot/gazebo_ros_control/pid_gains/*
  - /rrbot/rrbot/joint_limits/*
  This commit change these names to:
  - /rrbot/gazebo_ros_control/pid_gains/*
  - /rrbot/joint_limits/*
* Merge pull request #2 from ros-simulation/indigo-devel
  Indigo devel
* Merge pull request #322 from ros-simulation/issue_321
  Add ifdefs to fix build with gazebo2
* Add ifdefs to fix build with gazebo2
  It was broken by #315.
  Fixes #321.
* Merge pull request #315 from ros-simulation/max_force
  Use Joint::SetParam for joint velocity motors
* Merge pull request #314 from ros-simulation/gazebo_cpp11
  Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* Use Joint::SetParam for joint velocity motors
  Before gazebo5, Joint::SetVelocity and SetMaxForce
  were used to set joint velocity motors.
  The API has changed in gazebo5, to use Joint::SetParam
  instead.
  The functionality is still available through the SetParam API.
* Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* Contributors: Adolfo Rodriguez Tsouroukdissian, Akiyoshi Ochiai, Jose Luis Rivero, Steven Peters, iche033, ipa-fxm

2.4.8 (2015-03-17)
------------------
* Generate new changelog
* Merge pull request #244 from cottsay/control-urdf-fix
  gazebo_ros_control: add urdf to downstream catkin deps
* Merge pull request #283 from jim-rothrock/indigo-devel
  Added emergency stop support
* Added emergency stop support.
* Added emergency stop support.
* gazebo_ros_control: add urdf to downstream catkin deps
* Contributors: Adolfo Rodriguez Tsouroukdissian, Jim Rothrock, Jose Luis Rivero, Scott K Logan

2.4.7 (2014-12-15)
------------------
* Changelogs for 2.4.7 branch
* Merge pull request #266 from ipa-fxm/introduce_header_for_default_robot_hw_sim
  [gazebo_ros_control] move declaration for DefaultRobotHWSim to header file
* move declaration for DefaultRobotHWSim to header file
* Contributors: Adolfo Rodriguez Tsouroukdissian, Jose Luis Rivero, ipa-fxm

2.4.6 (2014-09-01)
------------------
* Changelogs for version 2.4.6
* 2.3.6
* Update changelogs for the upcoming release
* Merge pull request #221 from ros-simulation/fix_build
  Fix build for gazebo4
* Update default_robot_hw_sim.cpp
* Reduced changes
* Update for hydro + gazebo 1.9
* Fix to work with gazebo3
* Fix build with gazebo4 and indigo
* Update package.xml
  Add new maintainer.
* Merge remote-tracking branch 'origin/hydro-devel' into camera-info-manager
* Merge pull request #1 from ros-simulation/hydro-devel
  Merge from upstream
* Contributors: Adolfo Rodriguez Tsouroukdissian, Jonathan Bohren, Jose Luis Rivero, Nate Koenig, hsu, osrf

2.4.5 (2014-08-18)
------------------
* Changelogs for upcoming release
* Merge pull request #222 from ros-simulation/fix_build_indigo
  Port fix_build branch for indigo-devel (fix compilation for gazebo4)
* Fix typo: GAZEBO_VERSION_MAJOR -> GAZEBO_MAJOR_VERSION
* Port fix_build branch for indigo-devel
  See pull request #221
* Contributors: Jose Luis Rivero, Steven Peters, hsu

2.4.4 (2014-07-18)
------------------
* Update Changelog
* Update package.xml
  Add new maintainer.
* Merge pull request #217 from abubeck/patch-1
  Should fix build error for binary releases.
* Should fix build error for binary releases.
  See: http://www.ros.org/debbuild/indigo.html?q=gazebo_ros_control
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Updated package.xml
* Updated package.xml
* Merge pull request #207 from pal-robotics/fix-194-hydro
  gazebo_ros_control: Revert 4776545, as it belongs in indigo-devel.
* Merge pull request #208 from pal-robotics/fix-194-indigo
  gazebo_ros_control: Fix 194 indigo
* gazebo_ros_control: default_robot_hw_sim:  Suppressing pid error message
  Depends on ros-controls/control_toolbox#21
* Revert 4776545, as it belongs in indigo-devel.
* Merge pull request #194 from jbohren-forks/quiet-pid-check
  gazebo_ros_control: default_robot_hw_sim: Suppressing pid error message
* Merge pull request #201 from jonbinney/indigo-repos
  Fix repository urls for indigo branch
* Merge pull request #202 from jonbinney/hydro-repos
  Fix repo names in package.xml's (hydro-devel branch)
* Fix repo names in package.xml's
* Fix repo names in package.xml's
* gazebo_ros_control: default_robot_hw_sim: Suppressing pid error message, depends on ros-controls/control_toolbox#21
* Merge pull request #193 from cottsay/indigo-devel
  Fix build failures
* gazebo_ros_control: Add dependency on angles
* gazebo_ros_control: Add build-time dependency on gazebo
  This fixes a regression caused by a889ef8b768861231a67b78781514d834f631b8e
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Merge pull request #1 from ros-simulation/hydro-devel
  Merge from upstream
* Contributors: Adolfo Rodriguez Tsouroukdissian, Alexander Bubeck, Dave Coleman, Jon Binney, Jonathan Bohren, Markus Bader, Scott K Logan, Steven Peters

2.4.3 (2014-05-12)
------------------
* update changelog
* Merge pull request #185 from pal-robotics/gazebo-ros-control-indigo
  [gazebo_ros_control] Indigo compatibility
* Compatibility with Indigo's ros_control.
  Also fixes #184.
* Remove build-time dependency on gazebo_ros.
* Fix broken build due to wrong rosconsole macro use
* Contributors: Adolfo Rodriguez Tsouroukdissian, Steven Peters

2.4.2 (2014-03-27)
------------------
* catkin_tag_changelog
* catkin_generate_changelog
* merging from hydro-devel
* 2.3.5
* catkin_tag_changelog
* catkin_generate_changelog and fix rst format for forthcoming logs
* Merge pull request #135 from jim-rothrock/hydro-devel
  gazebo_ros_control: The position and velocity hardware interfaces are now fully supported.
* Removed some debugging code.
* joint->SetAngle() and joint->SetVelocity() are now used to control
  position-controlled joints and velocity-controlled joints that do not
  have PID gain values stored on the Parameter Server.
* Position-controlled and velocity-controlled joints now use PID controllers
  instead of calling SetAngle() or SetVelocity(). readSim() now longer calls
  angles::shortest_angular_distance() when a joint is prismatic.
  PLUGINLIB_EXPORT_CLASS is now used to register the plugin.
* gazebo_ros_control now depends on control_toolbox.
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Added support for the position hardware interface. Completed support for the
  velocity hardware interface.
* Removed the "support more hardware interfaces" line.
* Contributors: Dave Coleman, Jim Rothrock, John Hsu

2.4.1 (2013-11-13 18:52)
------------------------
* bump patch version for indigo-devel to 2.4.1
* merging from indigo-devel after 2.3.4 release
* "2.3.4"
* preparing for 2.3.4 release (catkin_generate_changelog, catkin_tag_changelog)
* Merge branch 'hydro-devel' of github.com:ros-simulation/gazebo_ros_pkgs into indigo-devel
* Merge pull request #144 from meyerj/fix-125
  Fixed #125: gazebo_ros_control: controlPeriod greater than the simulation period causes unexpected results
* Merge branch 'hydro-devel' into spawn_model_pose_fix
* Merge pull request #134 from meyerj/gazebo-ros-control-use-model-nh
  gazebo_ros_control: Use the model NodeHandle to get the robot_description parameter
* Merge pull request #131 from po1/fix-dep
  Fix dependency issues
* gazebo_ros_control: added GazeboRosControlPlugin::Reset() method that resets the timestamps on world reset
* gazebo_ros_control: call writeSim() for each Gazebo world update independent of the control period
* Merge pull request #143 from meyerj/patch-1
  gazebo_ros_pkgs: use GetMaxStepSize() for the Gazebo simulation period
* gazebo_ros_pkgs: use GetMaxStepSize() for the Gazebo simulation period
* gazebo_ros_control: use the model NodeHandle to get the robot_description parameter
* Add missing run_depend to urdf in gazebo_ros_control
* Remove dependency to meta-package ros_controllers
* Contributors: Johannes Meyer, John Hsu, Paul Mathieu, hsu

2.4.0 (2013-10-14)
------------------
* "2.4.0"
* catkin_generate_changelog
* Contributors: John Hsu

2.3.5 (2014-03-26)
------------------
* catkin_tag_changelog
* catkin_generate_changelog and fix rst format for forthcoming logs
* Merge pull request #135 from jim-rothrock/hydro-devel
  gazebo_ros_control: The position and velocity hardware interfaces are now fully supported.
* Removed some debugging code.
* joint->SetAngle() and joint->SetVelocity() are now used to control
  position-controlled joints and velocity-controlled joints that do not
  have PID gain values stored on the Parameter Server.
* Position-controlled and velocity-controlled joints now use PID controllers
  instead of calling SetAngle() or SetVelocity(). readSim() now longer calls
  angles::shortest_angular_distance() when a joint is prismatic.
  PLUGINLIB_EXPORT_CLASS is now used to register the plugin.
* gazebo_ros_control now depends on control_toolbox.
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Added support for the position hardware interface. Completed support for the
  velocity hardware interface.
* Removed the "support more hardware interfaces" line.
* Contributors: Dave Coleman, Jim Rothrock, John Hsu

2.3.4 (2013-11-13 18:05)
------------------------
* "2.3.4"
* preparing for 2.3.4 release (catkin_generate_changelog, catkin_tag_changelog)
* Merge pull request #144 from meyerj/fix-125
  Fixed #125: gazebo_ros_control: controlPeriod greater than the simulation period causes unexpected results
* Merge branch 'hydro-devel' into spawn_model_pose_fix
* Merge pull request #134 from meyerj/gazebo-ros-control-use-model-nh
  gazebo_ros_control: Use the model NodeHandle to get the robot_description parameter
* Merge pull request #131 from po1/fix-dep
  Fix dependency issues
* gazebo_ros_control: added GazeboRosControlPlugin::Reset() method that resets the timestamps on world reset
* gazebo_ros_control: call writeSim() for each Gazebo world update independent of the control period
* Merge pull request #143 from meyerj/patch-1
  gazebo_ros_pkgs: use GetMaxStepSize() for the Gazebo simulation period
* gazebo_ros_pkgs: use GetMaxStepSize() for the Gazebo simulation period
* gazebo_ros_control: use the model NodeHandle to get the robot_description parameter
* Add missing run_depend to urdf in gazebo_ros_control
* Remove dependency to meta-package ros_controllers
* Contributors: Johannes Meyer, John Hsu, Paul Mathieu, hsu

2.3.3 (2013-10-10)
------------------
* "2.3.3"
* preparing for 2.3.3 release (catkin_generate_changelog, catkin_tag_changelog)
* Merge pull request #119 from jim-rothrock/hydro-devel
  gazebo_ros_control now uses joint_limits_interface
* Eliminated a joint_name variable and replaced it with joint_names\_[j].
  Modified some lines so that they fit in 100 columns. These changes were made
  in order to be consistent with the rest of the file.
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* joint_limits_interface is now used to enforce limits on effort-controlled
  joints.
* Added "joint_limits_interface" and "urdf" to the component list.
* Additional parameters are passed to robot_hw_sim->initSim(). These parameters
  are used by the joint limits interface.
* Added "joint_limits_interface" and "urdf" to the build dependency list.
* Added the robot_namespace and urdf_model parameters to initSim().
* Added the urdf_string parameter to parseTransmissionsFromURDF().
* Contributors: Dave Coleman, Jim Rothrock, John Hsu

2.3.2 (2013-09-19)
------------------
* preparing for 2.3.2 release
* Merge pull request #114 from hsu/hydro-devel
  preparing for 2.3.2 release
* bump versions to 2.3.2
* Updating changelog for 2.3.2
* Merge branch 'hydro-devel' into synchronize_with_drcsim_plugins
* Contributors: John Hsu, hsu

2.3.1 (2013-08-27)
------------------
* Updating changelogs
* Merge pull request #103 from ros-simulation/ros_control_plugin_header
  Created a header file for the ros_control gazebo plugin
* Cleaned up template, fixes for header files
* Renamed plugin to match file name, tweaked CMakeLists
* Created a header file for the ros_control gazebo plugin
* Contributors: Dave Coleman, William Woodall

2.3.0 (2013-08-12)
------------------
* Updated changelogs
* Renamed ros_control_plugin, updated documentation
* Contributors: Dave Coleman

2.2.1 (2013-07-29 18:02)
------------------------
* Updated changelogs
* Contributors: Dave Coleman

2.2.0 (2013-07-29 13:55)
------------------------
* Updated changelogs
* Merge pull request #88 from ros-simulation/gazeb_plugins_ros_init
  Standardized the way ROS nodes are initialized in gazebo plugins
* Merged hydro branch
* Merge branch 'hydro-devel' into add_video_plugin
* Merged hydro-devel
* Merge pull request #87 from ros-simulation/remove_SDF_find_package_hydro
  Remove find_package(SDF) from CMakeLists.txt
* Standardized the way ROS nodes are initialized in gazebo plugins
* Remove find_package(SDF) from CMakeLists.txt
  It is sufficient to find gazebo, which will export the information
  about the SDFormat package.
* Merge branch 'hydro-devel' of github.com:ros-simulation/gazebo_ros_pkgs into hydro-pcl-conversions
* Merge pull request #80 from ros-simulation/tranmission_parsing
  Updated Tranmission parsing
* Merge branch 'tranmission_parsing' into groovy-devel
* Merge branch 'hydro-devel' into tranmission_parsing
* Merge branch 'hydro-devel' into merge_hydro_into_groovy
* Merged hydro-devel branch in groovy-devel
* Doc and debug update
* Merged hydro-devel
* Hid debug info
* Merged from Hydro-devel
* Merge branch 'hydro-devel' into tranmission_parsing
* Moved trasmission parsing to ros_control
* Contributors: Dave Coleman, John Hsu, Piyush Khandelwal, Steven Peters

2.1.5 (2013-07-18)
------------------
* changelogs for 2.1.5
* Contributors: Tully Foote

2.1.4 (2013-07-14)
------------------
* Bumped pkg version
* Updated changelogs
* Fixed for Jenkins broken dependency on SDF in ros_control
* Merge pull request #75 from ros-simulation/add_tbb_temp
  Add tbb temporarily to work around #74
* Contributors: Dave Coleman, Tully Foote

2.1.3 (2013-07-13)
------------------
* adding changelog 2.1.3
* Contributors: Tully Foote

2.1.2 (2013-07-12)
------------------
* Added changelogs
* Merge pull request #70 from ros-simulation/cmake_cleanup
  Cmake cleanup
* Cleaned up CMakeLists.txt for all gazebo_ros_pkgs
* Contributors: Dave Coleman

2.1.1 (2013-07-10)
------------------
* Merge pull request #66 from ros-simulation/dynamic_reconfigure
  Fixed dynamic reconfigure namespace, cleaned up various code
* Merge branch 'hydro-devel' into dev
* Merge pull request #64 from jhu-lcsr-forks/hydro-devel
  making RobotHWSim::initSim pure virtual
* making RobotHWSim::initSim pure virtual
* Cleaning up code
* Merge pull request #56 from jhu-lcsr-forks/hydro-devel
  Adding install targets
* Adding install targets
* Contributors: Dave Coleman, Jonathan Bohren

2.1.0 (2013-06-27)
------------------
* Made version match the rest of gazebo_ros_pkgs per bloom
* Added dependency on ros_controllers
* Merge branch 'hydro-devel' of github.com:osrf/gazebo_ros_pkgs into hydro-devel
* Clarifying language in readme
* Merge pull request #46 from osrf/robot_hw_sim
  <transmission> tags for gazebo_ros_control
* Made default period Gazebo's period
* Made control period optional
* Tweaked README
* Added support for reading <tranmission> tags and other cleaning up
* Merge pull request #44 from osrf/robot_hw_sim
  Renamed RobotSim to RobotHWSim
* Renamed RobotSim to RobotHWSim
* Merge branch 'hydro-devel' of https://github.com/osrf/gazebo_ros_pkgs into terminate_service_thread_fix
  Conflicts:
  gazebo_plugins/include/gazebo_plugins/PubQueue.h
* Merge pull request #40 from jhu-lcsr-forks/hydro-devel
  Eh, we don't have much time before Friday's freeze date.
  Adding merged gazebo_ros_control and ros_control_gazebo to gazebo_ros_pkgs
* Renaming all gazebo_ros_control stuff to be in the same package
* Refactoring gazebo_ros_control packages into a single package, removing exampls (they will go elsewhere)
* updating readme for gazebo_ros_control
* Merging in gazebo_ros_control
* making gazebo_ros_control a metapackage
* Moving readme
* Merging readmes
* eating this
* Merging gazebo_ros_control and ros_control_gazebo
* Contributors: Dave Coleman, Johannes Meyer, Jonathan Bohren

2.0.2 (2013-06-20)
------------------

2.0.1 (2013-06-19)
------------------

2.0.0 (2013-06-18)
------------------
