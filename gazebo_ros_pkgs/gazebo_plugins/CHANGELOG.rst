^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.7 (2018-01-12)
------------------

2.6.6 (2018-01-12)
------------------
* fixed unitialized values
* Contributors: Hilario Tome

2.6.5 (2018-01-11)
------------------
* removed changelogs and unified package versions
* Fix gazebo8 warnings part 8: ifdef's for GetWorldPose (#650)
  * gazebo_ros_vacuum_gripper: ifdef one GetWorldPose
  * ifdef all remaining GetWorldPose calls
  ~~~
  sed -i -e 's@.*GetWorldPose.*@#if GAZEBO_MAJOR_VERSION >= 8\
  __REPLACE_\_&\
  \#else\
  &\
  \#endif
  ~~~
  ~~~
  sed -i -e \
  's@^__REPLACE_\_\(.*\)GetWorldPose()\.Ign\(.*\)@\1WorldPose\2@' \
  $(grep -rlI GetWorldPose gazebo\_* | grep -v vacuum)
  ~~~
  * remove ifdefs for commented GetWorldPose
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
* for gazebo8+, call functions without Get (#639)
* Fix gazebo8 warnings part 4: convert remaining local variables in plugins to ign-math (#633)
  * plugins: convert all local vars to ign-math
  * ft_sensor: fix gazebo7 build
  * Use World::[GS]etGravity
  * fix gravity syntax
* Fix gazebo8 warnings part 3: more ign-math in plugins (#631)
  * gazebo_plugins: more conversion to ign-math
  * gazebo_plugins replace gazebo/math headers
  * diff_drive plugin: convert types to ignition math
  * skid_steer plugin: convert types to ignition math
  * tricycle plugin: convert types to ignition math
* Fix gazebo8 warnings part 2: replace private member gazebo::math types with ignition (#628)
  * Remove old compiler directive blocks
  * gazebo_ros_imu_sensor: convert to ignition math
  * gazebo_ros_planar_move: ign private/local vars
  * gazebo_ros_imu: ign private/local vars
  * gazebo_ros_p3d: ign private/local vars
* Replace Events::Disconnect* with pointer reset (#623)
* Merge pull request #542 from davetcoleman/kinetic-gazebo7-only
  Remove compiler directive flags for < GAZEBO 7
* Remove compiler directive flags for < GAZEBO 7
* Contributors: Dave Coleman, Jose Luis Rivero, Steven Peters

2.5.13 (2017-06-24)
-------------------
* Update changelogs
* Fix inverted height in block laser plugin (#582)
* Allow disabling distorted camera border crop (and associated tests) (#572)
  Allow disabling distorted camera border crop
* Add an IMU sensor plugin that inherits from SensorPlugin (#363)
  * added a IMU sensor plugin that inherits from SensorPlugin
  * now the plugin works with multiple robots
  * using GetParentName name instead of GetScopedName
  * added comments to highlight the differents between GazeboRosImuSensor and GazeboRosIMU
  * now the message header is properly handled, using bodyName parameter as frame_id
  * added check on gazebo version
  * added check for sensor null pointer
  * changed deprecated functions for gazebo version >= 6
  * fixed version check
  * added missing sensor variable for LastUpdateTime() function call
  * considering '/' included in the robotNamespace
  * replaced "bodyFrame" with "frameName"
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
* Contributors: Adam Allevato, Alessandro Settimi, Dave Coleman, Jose Luis Rivero, Shohei Fujii

2.5.12 (2017-04-25)
-------------------
* Changelogs for next version
* Revert catkin warning fix (#567)
  Many regressions in third party software (see https://github.com/yujinrobot/kobuki_desktop/issues/50)
* Contributors: Jose Luis Rivero

2.5.11 (2017-04-18)
-------------------
* Changelogs to prepare for next 2.5.11
* Change build system to set DEPEND on Gazebo/SDFormat (fix catkin warning)
  Added missing DEPEND clauses to catkin_package to fix gazebo catkin warning. Note that after the change problems could appear related to -lpthreads errors. This is an known issue related to catkin: https://github.com/ros/catkin/issues/856.
* Fix: add gazebo_ros_range to catkin package libraries (#558)
* Contributors: Christoph Rist, Dave Coleman, Jose Luis Rivero

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
* Fix destructor of GazeboRosVideo (#547)
* Less exciting console output (#549)
* Fix SDF namespacing for Video Plugin (#546)
* Contributors: Dave Coleman, Jose Luis Rivero

2.5.9 (2017-02-20)
------------------
* Update changelogs
* Fix gazebo catkin warning, cleanup CMakeLists (#537)
* Merge pull request #545 from ros-simulation/kinetic-devel_transplant_538
  Fix timestamp issues for rendering sensors (kinetic-devel)
* Fix timestamp issues for rendering sensors (kinetic-devel)
  This PR builds on top of pull request #410 and applies the timestamp fix
  to kinect_openni and prosilica sensors
* Namespace console output (#543)
  Namespace all console output
* Merge pull request #540 from ros-simulation/kinetic-devel-transplant-410
  Correct the timestamp used by the camera (kinetic-devel)
* Fix problem introduced with the merge
* Fix merge with kinetic branch
* #408 Increasing max time because some systems are taking 0.6 seconds to receive the messages (still well less than 2.0 seconds). Also all the tests can be run with run_tests_gazebo_plugins_rostest but only with the -j1 flag #409
* Fix merge with kinetic branch
* Fix merge with kinetic branch
* #408 also test points publication
* #408 Created test for depth camera, which fails, so next make it pass
* Disabling this test because of #409
* Adding depth camera world to use in test to make depth camera have right timestamp #408- appears to be working (though only looking at horizon) but getting these sdf errors:
  Error [SDF.cc:789] Missing element description for [pointCloudTopicName]
  Error [SDF.cc:789] Missing element description for [depthImageCameraInfoTopicName]
  Error [SDF.cc:789] Missing element description for [pointCloudCutoff]
* #408 Make the multi camera timestamps current rather than outdated, also reuse the same update code
* Fix merge with kinetic branch
* #408 Making a test for multicamra that shows the timestamps are currently outdated, will fix them similar to how the regular camera was fixed.
* Fix for issue #408. The last measurement time is the time that gazebo generated the sensor data, so ought to be used. updateRate doesn't seem that useful.
  The other cameras need similar fixes to have the proper timestamps.
* Fix merge with kinetic branch
* Merge pull request #539 from davetcoleman/kinetic-whitespace
  Removed all trailing whitespace
* Removed all trailing whitespace
* Merge pull request #534 from IanTheEngineer/fix-camera-util-cp
  [gazebo_plugins] bugfix: duplicated tf prefix resolution (kinetic-devel)
* Merge pull request #521 from ros-simulation/fix_warnings
  Fix gazebo and sdformat catkin warnings
* [gazebo_plugins] bugfix: duplicated tf prefix resolution
  (cherry picked from commit d760220bfb28e639f28fa933edf315699127dcd0)
* Merge pull request #522 from ros-simulation/kinetic-devel-transplant-492
  fill in child_frame_id of odom topic (kinetic-devel)
* fill in child_frame_id of odom topic
* Use uppercase to workaround the catkin warning
* Fix gazebo and sdformat catkin warnings
* Contributors: Dave Coleman, Jose Luis Rivero, Kei Okada, Lucas Walter, Yuki Furuta

2.5.8 (2016-12-06)
------------------
* Update changelogs for 2.5.8
* Merge pull request #505 from ros-simulation/kinetic-devel-transplant-503
  Fix distortion coefficients order (kinetic-devel)
* Fix distortion coefficients order
  It should be D = {k1, k2, p1, p2, k3}, according to:
  - sensor_msgs/CameraInfo:
  http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  - OpenCV:
  http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
* Use NOT VERSION_LESS to simplify cmake logic
* Added an interface to gazebo's harness plugin
* Contributors: Enrique Fernandez, Jose Luis Rivero, Steven Peters, nate koenig

2.5.7 (2016-06-10)
------------------
* Update changelogs
* Contributors: Jose Luis Rivero

2.5.6 (2016-04-28)
------------------
* Fix versions in CHANGELOG
* 2.5.5
* Update changelogs
* Merge pull request #455 from scpeters/warnings_kinetic
  fix gazebo7 deprecation warnings on kinetic
* fix gazebo7 deprecation warnings on kinetic
* Contributors: Jose Luis Rivero, Steven Peters

2.5.4 (2016-04-27)
------------------
* Update changelogs
* Merge pull request #454 from scpeters/merge_ijk
  merge indigo, jade to kinetic-devel
* merge indigo, jade to kinetic-devel
* Merge pull request #437 from ros-simulation/fix_issue_324
  issue #324 Also accept /world for the frameName parameter in gazebo_r…
* issue #324 Also accept /world for the frameName parameter in gazebo_ros_p3d
* Revert "issue #324 Also accept /world for the frameName parameter in gazebo_ros_p3d"
  This reverts commit 962e7b48ab1d59fd42c09078c2721b0d3b172b9c.
* issue #324 Also accept /world for the frameName parameter in gazebo_ros_p3d
* Merge branch 'kinetic-devel' of https://github.com/ros-simulation/gazebo_ros_pkgs into kinetic-devel
* Upgrade to gazebo 7 and remove deprecated driver_base dependency (#426)
  * Upgrade to gazebo 7 and remove deprecated driver_base dependency
  * disable gazebo_ros_control until dependencies are met
  * Remove stray backslash
* Merge pull request #430 from ros-simulation/kinetic-devel-maintainer
  Update maintainer for Kinetic release
* Update maintainer for Kinetic release
* use HasElement in if condition
* Contributors: Hugo Boyer, Jackie Kay, Jose Luis Rivero, Steven Peters, William Woodall, Yuki Furuta

2.5.3 (2016-04-11)
------------------
* Update changelogs for 2.5.3
* Merge branch 'jade-devel' into issue_387_remove_ros_remappings
* Contributors: Jose Luis Rivero, Martin Pecka

2.5.2 (2016-02-25)
------------------
* Prepare changelogs
* Merge pull request #391 from wkentaro/fix-openni-row-step
  [gazebo_plugins] Fix row_step of openni_kinect plugin
* Fix row_step of openni_kinect plugin
* remove duplicated code during merge
* merging from indigo-devel
* Merge pull request #357 from MirkoFerrati/indigo-devel
  Minor: Added a missing variable initialization inside Differential Drive
* Merge pull request #368 from l0g1x/jade-devel
  Covariance for published twist in skid steer plugin
* Merge pull request #373 from wkentaro/openni-kinect-organized-points
  [gazebo_plugins] Publish organized point cloud from openni_kinect plugin
* gazebo_ros_utils.h: include gazebo_config.h
  Make sure to include gazebo_config.h,
  which defines the GAZEBO_MAJOR_VERSION macro
* Fix compiler error with SetHFOV
  In gazebo7, the rendering::Camera::SetHFOV function
  is overloaded with a potential for ambiguity,
  as reported in the following issue:
  https://bitbucket.org/osrf/gazebo/issues/1830
  This fixes the build by explicitly defining the
  Angle type.
* Add missing boost header
  Some boost headers were remove from gazebo7 header files
  and gazebo_ros_joint_state_publisher.cpp was using it
  implicitly.
* Fix gazebo7 build errors
  The SensorPtr types have changed from boost:: pointers
  to std:: pointers,
  which requires boost::dynamic_pointer_cast to change to
  std::dynamic_pointer_cast.
  A helper macro is added that adds a `using` statement
  corresponding to the correct type of dynamic_pointer_cast.
  This macro should be narrowly scoped to protect
  other code.
* Merge pull request #381 from ros-simulation/gazebo7_fixes
  Gazebo7 fixes
* gazebo_ros_utils.h: include gazebo_config.h
  Make sure to include gazebo_config.h,
  which defines the GAZEBO_MAJOR_VERSION macro
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
* Fix gazebo6 deprecation warnings
  Several RaySensor functions are deprecated in gazebo6
  and are removed in gazebo7.
  The return type is changed to use ignition math
  and the function name is changed.
  This adds ifdef's to handle the changes.
* Merge pull request #380 from ros-simulation/gazebo6_angle_deprecations
  Fix gazebo6 deprecation warnings
* Fix compiler error with SetHFOV
  In gazebo7, the rendering::Camera::SetHFOV function
  is overloaded with a potential for ambiguity,
  as reported in the following issue:
  https://bitbucket.org/osrf/gazebo/issues/1830
  This fixes the build by explicitly defining the
  Angle type.
* Add missing boost header
  Some boost headers were remove from gazebo7 header files
  and gazebo_ros_joint_state_publisher.cpp was using it
  implicitly.
* Fix gazebo7 build errors
  The SensorPtr types have changed from boost:: pointers
  to std:: pointers,
  which requires boost::dynamic_pointer_cast to change to
  std::dynamic_pointer_cast.
  A helper macro is added that adds a `using` statement
  corresponding to the correct type of dynamic_pointer_cast.
  This macro should be narrowly scoped to protect
  other code.
* Fix gazebo6 deprecation warnings
  Several RaySensor functions are deprecated in gazebo6
  and are removed in gazebo7.
  The return type is changed to use ignition math
  and the function name is changed.
  This adds ifdef's to handle the changes.
* Publish organized point cloud from openni_kinect plugin
* Added covariance matrix for published twist message in the skid steer plugin, as packages such as robot_localization require an associated non-zero covariance matrix
* Added a missing initialization inside Differential Drive
* 2.4.9
* Generate changelog
* Merge pull request #335 from pal-robotics-forks/add_range_sensor_plugin
  Adds range plugin for infrared and ultrasound sensors from PAL Robotics
* Merge pull request #350 from ros-simulation/indigo-devel_merged_from_jade
  Merge changes from jade-devel into indigo-devel
* Import changes from jade-branch
* Add range world and launch file
* Adds range plugin for infrared and ultrasound sensors from PAL Robotics
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
* Contributors: Bence Magyar, John Hsu, Jose Luis Rivero, Kentaro Wada, Krystian, Mirko Ferrati, Steven Peters, hsu, iche033

2.5.1 (2015-08-16 02:31)
------------------------
* Generate changelogs
* Merge pull request #352 from ros-simulation/add_range_sensor_plugin-jade
  Port of Pal Robotics range sensor plugin to Jade
* Port of Pal Robotics range sensor plugin to Jade
* Merge pull request #338 from ros-simulation/elevator
  Elevator plugin
* Merge pull request #330 from ros-simulation/issue_323
  run_depend on libgazebo5-dev (#323)
* Added a comment about the need of libgazebo5-dev in runtime
* Added gazebo version check
* Added missing files
* Added elevator plugin
* Merge pull request #336 from ros-simulation/jade-devel-c++11
  Use c++11
* Use c++11
* run_depend on libgazebo5-dev (#323)
  Declare the dependency.
  It can be fixed later if we don't want it.
* Contributors: Jose Luis Rivero, Nate Koenig, Steven Peters

2.5.0 (2015-04-30)
------------------
* changelogs
* run_depend on libgazebo5-dev instead of gazebo5
* changelogs
* change the rosdep key for gazebo to gazebo5
* Contributors: Steven Peters, William Woodall

2.4.9 (2015-08-16 01:30)
------------------------
* Generate changelog
* Merge pull request #335 from pal-robotics-forks/add_range_sensor_plugin
  Adds range plugin for infrared and ultrasound sensors from PAL Robotics
* Merge pull request #350 from ros-simulation/indigo-devel_merged_from_jade
  Merge changes from jade-devel into indigo-devel
* Import changes from jade-branch
* Add range world and launch file
* Adds range plugin for infrared and ultrasound sensors from PAL Robotics
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
* Contributors: Bence Magyar, Jose Luis Rivero, Steven Peters, iche033

2.4.8 (2015-03-17)
------------------
* Generate new changelog
* Merge pull request #296 from mikeferguson/indigo-devel
  add PointCloudCutoffMax
* Merge pull request #298 from k-okada/reset_diff_drive
  [gazebo_ros_diff_drive] force call SetMaxForce
* Merge pull request #299 from sabrina-heerklotz/indigo-devel
  fixed mistake at calculation of joint velocity
* fixed mistake at calculation of joint velocity
* [gazebo_ros_diff_drive] force call SetMaxForce since this Joint::Reset in gazebo/physics/Joint.cc reset MaxForce to zero and ModelPlugin::Reset is called after Joint::Reset
* add PointCloudCutoffMax
* Contributors: Jose Luis Rivero, Kei Okada, Michael Ferguson, Sabrina Heerklotz, hsu

2.4.7 (2014-12-15)
------------------
* Changelogs for 2.4.7 branch
* Merge pull request #275 from ros-simulation/opencv_resize
  change header to use opencv2/opencv.hpp issue #274
* Merge pull request #255 from ros-simulation/fix_gazebo_ros_tutorial_url
  Update Gazebo/ROS tutorial URL
* Merge pull request #276 from ros-simulation/gazebo_ogre_compile_flag_fix
  fix missing ogre flags: removed from gazebo default (5.x.x candidate) cmake config
* Merge pull request #238 from ayrton04/indigo-devel
  Fixing handling of non-world frame velocities in setModelState.
* fix missing ogre flags (removed from gazebo cmake config)
* change header to use opencv2/opencv.hpp issue #274
* Merge pull request #271 from jhu-lcsr-forks/indigo-devel
  gazebo_plugins: Adding ogre library dirs to cmakelists
* Update CMakeLists.txt
* Fixing set model state method and test
* Merge branch 'indigo-devel' into patch-1
* Adding test for set_model_state
* Update Gazebo/ROS tutorial URL
* Merge pull request #241 from ros-simulation/fix_compiler_warning_gazebo_ros_diff_drive
  fix compiler warning
* Merge pull request #237 from ros-simulation/update_header_license
  Update header license for Indigo
* fix compiler warning
* update headers to apache 2.0 license
* update headers to apache 2.0 license
* Contributors: John Hsu, Jonathan Bohren, Jose Luis Rivero, Martin Pecka, Robert Codd-Downey, Tom Moore, hsu

2.4.6 (2014-09-01)
------------------
* Changelogs for version 2.4.6
* Merge pull request #233 from ros-simulation/merge-hydro-devel-to-indigo-devel
  Merge hydro devel to indigo devel
* Update gazebo_ros_openni_kinect.cpp
* fix merge
* merging from hydro-devel into indigo-devel
* Merge pull request #204 from fsuarez6/hydro-devel
  gazebo_plugins: Adding ForceTorqueSensor Plugin
* Merge pull request #229 from ros-simulation/fix_build
  check deprecation of gazebo::Joint::SetAngle by SetPosition in gazebo 4.0
* Updated to Apache 2.0 license
* Merge branch 'jbohren-forks-camera-info-manager' into hydro-devel
* merging from hydro-devel
* Merge pull request #211 from garaemon/organized-openni-pointcloud
  publish organized pointcloud from openni plugin
* Merge pull request #205 from fsuarez6/imu-plugin
  gazebo_plugins: Added updateRate parameter to the gazebo_ros_imu plugin
* Merge pull request #231 from ros-simulation/fix_bad_merge_diff_drive
  fix bad merge
* fix bad merge
* Merge pull request #180 from vrabaud/indigo-devel
  remove PCL dependency
* Merge pull request #230 from ros-simulation/curranw-hydro-devel
  merging pull request #214
* fix style
* merging
* check deprecation of gazebo::Joint::SetAngle by SetPosition
* compatibility with gazebo 4.x
* 2.3.6
* Update changelogs for the upcoming release
* Merge pull request #221 from ros-simulation/fix_build
  Fix build for gazebo4
* Fix build with gazebo4 and indigo
* Merge pull request #1 from gborque/hydro-devel
  Added Gaussian Noise generator
* Added Gaussian Noise generator
* publish organized pointcloud from openni plugin
* Changed measurement direction to "parent to child"
* Included changes suggested by @jonbinney
* gazebo_plugin: Added updateRate parameter to the gazebo_ros_imu plugin
* Added description and example usage in the comments
* gazebo_plugins: Adding ForceTorqueSensor Plugin
* remove PCL dependency
* Merge remote-tracking branch 'origin/hydro-devel' into camera-info-manager
* Merge pull request #1 from ros-simulation/hydro-devel
  Merge from upstream
* ros_camera_utils: Adding CameraInfoManager to satisfy full ROS camera API (relies on https://github.com/ros-perception/image_common/pull/20 )
  ros_camera_utils: Adding CameraInfoManager to satisfy full ROS camera API (relies on https://github.com/ros-perception/image_common/pull/20 )
* Contributors: Francisco, John Hsu, Jonathan Bohren, Jose Luis Rivero, Nate Koenig, Ryohei Ueda, Vincent Rabaud, fsuarez6, gborque, hsu

2.4.5 (2014-08-18)
------------------
* Changelogs for upcoming release
* Merge pull request #222 from ros-simulation/fix_build_indigo
  Port fix_build branch for indigo-devel (fix compilation for gazebo4)
* Replace SetAngle with SetPosition for gazebo 4 and up
* Port fix_build branch for indigo-devel
  See pull request #221
* Contributors: Jose Luis Rivero, Steven Peters, hsu

2.4.4 (2014-07-18)
------------------
* Update Changelog
* Merge branch 'hydro-devel' into indigo-devel
* Merge pull request #141 from moresun/hydro-devel
  Gazebo ROS joint state publisher added
* gazebo_ros_diff_drive gazebo_ros_tricycle_drive encoderSource option names updated
* gazebo_ros_diff_drive is now able to use the wheels rotation of the optometry or the gazebo ground truth based on the 'odometrySource' parameter
* minor fix
* simple linear controller for the tricycle_drive added
* second robot for testing in tricycle_drive_scenario.launch added
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* BDS licenses header fixed and tricycle drive plugin added
* format patch of hsu applied
* Updated package.xml
* Updated package.xml
* Merge pull request #201 from jonbinney/indigo-repos
  Fix repository urls for indigo branch
* Merge pull request #202 from jonbinney/hydro-repos
  Fix repo names in package.xml's (hydro-devel branch)
* Fix repo names in package.xml's
* Fix repo names in package.xml's
* ros diff drive supports now an acceleration limit
* Merge pull request #191 from jbohren-forks/indigo-devel
  adding hand-of-god plugin to indigo
* Pioneer model: Diff_drive torque reduced
* GPU Laser test example added
* fixed gpu_laser to work with workspaces
* HoG: adding install target
* hand_of_god: Adding hand-of-god plugin
  ros_force: Fixing error messages to refer to the right plugin
* Merge pull request #139 from jbohren-forks/hand-of-god
  Adding hand-of-god plugin
* HoG: adding install target
* hand_of_god: Adding hand-of-god plugin
  ros_force: Fixing error messages to refer to the right plugin
* Remove unneeded dependency on pcl_ros
  pcl_ros hasn't been released yet into indigo. I asked @wjwwood about
  its status, and he pointed out that our dependency on pcl_ros
  probably isn't necessary. Lo and behold, we removed it from the
  header files, package.xml and CMakeLists.txt and gazebo_plugins
  still compiles.
* minor fixes on relative paths in xacro for pioneer robot
* gazebo test model pionneer 3dx updated with xacro path variables
* pioneer model update for the multi_robot_scenario
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* fixed camera to work with workspaces
* fixed camera to work with workspaces
* fixed links related to changed name
* diff drive name changed to multi robot scenario
* working camera added
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* fix in pioneer xacro model for diff_drive
* Laser colour in rviz changed
* A test model for the ros_diff_drive ros_laser and joint_state_publisher added
* the ros_laser checkes now for the model name and adds it als prefix
* joint velocity fixed using radius instead of diameter
* Merge pull request #1 from ros-simulation/hydro-devel
  Merge from upstream
* ROS_INFO on laser plugin added to see if it starts
* fetched with upstream
* gazebo_ros_diff_drive was enhanced to publish the wheels tf or the wheels joint state depending on two additinal xml options <publishWheelTF> <publishWheelJointState>
* Gazebo ROS joint state publisher added
* Contributors: Dave Coleman, John Hsu, Jon Binney, Jonathan Bohren, Markus Bader, Steven Peters, hsu

2.4.3 (2014-05-12)
------------------
* update changelog
* Merge pull request #181 from ros-simulation/gazebo_plugins_undepend
  Reverse gazebo_ros dependency on gazebo_plugins
* gazebo_plugins: add run-time dependency on gazebo_ros
* Merge pull request #176 from ros-simulation/issue_175
  Fix #175: dynamic reconfigure dependency error
* Merge pull request #177 from ros-simulation/pcl_ros_undepend
  Remove unneeded dependency on pcl_ros
* Remove unneeded dependency on pcl_ros
  pcl_ros hasn't been released yet into indigo. I asked @wjwwood about
  its status, and he pointed out that our dependency on pcl_ros
  probably isn't necessary. Lo and behold, we removed it from the
  header files, package.xml and CMakeLists.txt and gazebo_plugins
  still compiles.
* Fix #175: dynamic reconfigure dependency error
* Contributors: Dave Coleman, Steven Peters

2.4.2 (2014-03-27)
------------------
* catkin_tag_changelog
* catkin_generate_changelog
* merging from hydro-devel
* 2.3.5
* catkin_tag_changelog
* catkin_generate_changelog and fix rst format for forthcoming logs
* Merge pull request #171 from pal-robotics/fix-multicamera
  multicamera bad namespace. Fixes #161
* Merge pull request #172 from toliver/F_fix_kinect_depth_image_publish
  Initialize depth_image_connect_count\_ in openni_kinect plugin
* update test world for block laser
* this corrects the right orientation of the laser scan and improves on comparison between 2 double numbers
* Initialize depth_image_connect_count\_ in openni_kinect plugin
* multicamera bad namespace. Fixes #161
  There was a race condition between GazeboRosCameraUtils::LoadThread
  creating the ros::NodeHandle and GazeboRosCameraUtils::Load
  suffixing the camera name in the namespace
* Merge pull request #167 from iche033/hydro-devel
  Replace reference to `sceneNode` with function call in gazebo_ros_video
* Use function for accessing scene node in gazebo_ros_video
* Merge pull request #156 from shadow-robot/fix_gazebo_plugins_bumper
  [gazebo_plugins] Fix gazebo plugins bumper
* readded the trailing whitespace for cleaner diff
* the parent sensor in gazebo seems not to be active
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Contributors: Dejan Pangercic, Ian Chen, Jim Rothrock, John Hsu, Jordi Pages, Toni Oliver, Ugo Cupcic, hsu

2.4.1 (2013-11-13 18:52)
------------------------
* bump patch version for indigo-devel to 2.4.1
* merging from indigo-devel after 2.3.4 release
* "2.3.4"
* preparing for 2.3.4 release (catkin_generate_changelog, catkin_tag_changelog)
* Merge branch 'hydro-devel' of github.com:ros-simulation/gazebo_ros_pkgs into indigo-devel
* Merge pull request #128 from ros-simulation/cmake_fixes
  Some fixes and simplifications of gazebo_plugins/CMakeLists.txt
* Simplify gazebo_plugins/CMakeLists.txt
  Replace cxx_flags and ld_flags variables with simpler cmake macros
  and eliminate unnecessary references to SDFormat_LIBRARIES, since
  they are already part of GAZEBO_LIBRARIES.
* Put some cmake lists on multiple lines to improve readability.
* Add dependencies on dynamic reconfigure files
  Occasionally the build can fail due to some targets having an
  undeclared dependency on automatically generated dynamic
  reconfigure files (GazeboRosCameraConfig.h for example). This
  commit declares several of those dependencies.
* Contributors: John Hsu, Steven Peters, hsu

2.4.0 (2013-10-14)
------------------
* "2.4.0"
* catkin_generate_changelog
* Contributors: John Hsu

2.3.5 (2014-03-26)
------------------
* catkin_tag_changelog
* catkin_generate_changelog and fix rst format for forthcoming logs
* Merge pull request #171 from pal-robotics/fix-multicamera
  multicamera bad namespace. Fixes #161
* Merge pull request #172 from toliver/F_fix_kinect_depth_image_publish
  Initialize depth_image_connect_count\_ in openni_kinect plugin
* update test world for block laser
* this corrects the right orientation of the laser scan and improves on comparison between 2 double numbers
* Initialize depth_image_connect_count\_ in openni_kinect plugin
* multicamera bad namespace. Fixes #161
  There was a race condition between GazeboRosCameraUtils::LoadThread
  creating the ros::NodeHandle and GazeboRosCameraUtils::Load
  suffixing the camera name in the namespace
* Merge pull request #167 from iche033/hydro-devel
  Replace reference to `sceneNode` with function call in gazebo_ros_video
* Use function for accessing scene node in gazebo_ros_video
* Merge pull request #156 from shadow-robot/fix_gazebo_plugins_bumper
  [gazebo_plugins] Fix gazebo plugins bumper
* readded the trailing whitespace for cleaner diff
* the parent sensor in gazebo seems not to be active
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Contributors: Dejan Pangercic, Ian Chen, Jim Rothrock, John Hsu, Jordi Pages, Toni Oliver, Ugo Cupcic, hsu

2.3.4 (2013-11-13 18:05)
------------------------
* "2.3.4"
* preparing for 2.3.4 release (catkin_generate_changelog, catkin_tag_changelog)
* Merge pull request #128 from ros-simulation/cmake_fixes
  Some fixes and simplifications of gazebo_plugins/CMakeLists.txt
* Simplify gazebo_plugins/CMakeLists.txt
  Replace cxx_flags and ld_flags variables with simpler cmake macros
  and eliminate unnecessary references to SDFormat_LIBRARIES, since
  they are already part of GAZEBO_LIBRARIES.
* Put some cmake lists on multiple lines to improve readability.
* Add dependencies on dynamic reconfigure files
  Occasionally the build can fail due to some targets having an
  undeclared dependency on automatically generated dynamic
  reconfigure files (GazeboRosCameraConfig.h for example). This
  commit declares several of those dependencies.
* Contributors: John Hsu, Steven Peters, hsu

2.3.3 (2013-10-10)
------------------
* "2.3.3"
* preparing for 2.3.3 release (catkin_generate_changelog, catkin_tag_changelog)
* Merge pull request #120 from meyerj/fix-gazebo-plugins-segfaults
  Segfaults in camera gazebo plugins due to uninitialized shared pointers
* gazebo_plugins: use shared pointers for variables shared among cameras
  It is not allowed to construct a shared_ptr from a pointer to a member
  variable.
* gazebo_plugins: moved initialization of shared_ptr members of
  GazeboRosCameraUtils to GazeboRosCameraUtils::Load()
  This fixes segfaults in gazebo_ros_depth_camera and
  gazebo_ros_openni_kinect as the pointers have not been initialized
  there.
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Merge branch 'hydro-devel' of github.com:ros-simulation/gazebo_ros_pkgs into hydro-devel
* Merge pull request #117 from ros-simulation/rendering_fix_hydro
  Use RenderingIFace.hh
* Use RenderingIFace.hh
* Contributors: Dave Coleman, Jim Rothrock, Johannes Meyer, John Hsu, Nate Koenig, hsu, nkoenig

2.3.2 (2013-09-19)
------------------
* preparing for 2.3.2 release
* Merge pull request #114 from hsu/hydro-devel
  preparing for 2.3.2 release
* bump versions to 2.3.2
* Updating changelog for 2.3.2
* Merge pull request #109 from hsu/hydro-devel-gazebo-paging-support
  add OGRE-Paging as dependency since gazebo is doing paging.
* switch from OGRE-Paging to OGRE-Terrain per pull request comment
* Merge pull request #113 from dirk-thomas/hydro-devel
  add missing build dependency on diagnostic_updater
* add missing build dependency on diagnostic_updater which is used in src/gazebo_ros_prosilica.cpp
* Fix openni plugin
* add OGRE-Paging as dependency since gazebo is doing paging.
* Merge pull request #104 from ros-simulation/synchronize_with_drcsim_plugins
  synchronize with drcsim plugins
* Merge pull request #108 from ros-simulation/fix_gazebo_includes
  Make gazebo includes use full path
* Make gazebo includes use full path
  In the next release of gazebo, it will be required to use the
  full path for include files. For example,
  include <physics/physics.hh> will not be valid
  include <gazebo/physics/physics.hh> must be done instead.
* Merge branch 'hydro-devel' of github.com:ros-simulation/gazebo_ros_pkgs into synchronize_with_drcsim_plugins
* Merge pull request #105 from fmder/camera-util-robotnamespace
  Camera util cannot find tf_prefix
* change includes to use brackets in headers for export
* Merge branch 'hydro-devel' into synchronize_with_drcsim_plugins
* per pull request comments
* Changed resolution for searchParam.
* Don't forget to delete the node!
* Removed info message on robot namespace.
* Retreive the tf prefix from the robot node.
* synchronize with drcsim plugins
* Contributors: Dirk Thomas, François-Michel De Rainville, John Hsu, Steven Peters, hsu, nkoenig

2.3.1 (2013-08-27)
------------------
* Updating changelogs
* Remove direct dependency on pcl, rely on the transitive dependency from pcl_ros
* Merge pull request #103 from ros-simulation/ros_control_plugin_header
  Created a header file for the ros_control gazebo plugin
* Cleaned up template, fixes for header files
* Contributors: Dave Coleman, William Woodall

2.3.0 (2013-08-12)
------------------
* Updated changelogs
* Merge pull request #101 from piyushk/fix-openni-rgb-in-cloud
  Fix rgb in cloud in openni_kinect
* Merge branch 'hydro-devel' of https://github.com/ros-simulation/gazebo_ros_pkgs into hydro-devel
* enable image generation when pointcloud is requested, as the generated image is used by the pointcloud
* Merge pull request #97 from bit-pirate/hydro-devel
  gazebo_ros_openni_kinect plugin: adds publishing of the camera info again (fixes #95)
* Merge pull request #100 from ros-simulation/fix_osx
  Fixes found while building on OS X
* gazebo_plugins: replace deprecated boost function
  This is related to this gazebo issue:
  https://bitbucket.org/osrf/gazebo/issue/581/boost-shared\_-_cast-are-deprecated-removed
* gazebo_plugins: fix linkedit issues
  Note: other linkedit errors were fixed upstream
  in gazebo
* gazebo_ros_openni_kinect plugin: adds publishing of the camera info
  again (fixes #95)
* Merge pull request #90 from piyushk/add_model_controller
  added a simple model controller plugin that uses a twist message
* renamed plugin from model controller to planar move
* Merge pull request #96 from bit-pirate/hydro-devel
  prevents dynamic_reconfigure from overwritting update rate param on start-up
* prevents dynamic_reconfigure from overwritting update rate param on start-up
* removed anonymizer from include guard
* fixed odometry publication for model controller plugin
* added a simple model controller plugin that uses a twist message to control models
* Contributors: Dave Coleman, Marcus Liebhardt, Piyush Khandelwal, William Woodall

2.2.1 (2013-07-29 18:02)
------------------------
* Updated changelogs
* Added prosilica plugin to install TARGETS
* Contributors: Dave Coleman

2.2.0 (2013-07-29 13:55)
------------------------
* Updated changelogs
* Switched to pcl_conversions
* Merge pull request #88 from ros-simulation/gazeb_plugins_ros_init
  Standardized the way ROS nodes are initialized in gazebo plugins
* Merged hydro branch
* Merge pull request #89 from ros-simulation/hydro-pcl-conversions
  Add Grooby pcl_conversions
* Merge pull request #86 from piyushk/add_video_plugin
  ROS Video Plugin for Gazebo - allows displaying an image stream in an OG...
* fixed node intialization conflict between gzserver and gzclient. better adherance to gazebo style guidelines
* Fixed template
* Merge branch 'hydro-devel' into add_video_plugin
* removed ros initialization from plugin
* Added back PCL dependency
* Merged hydro-devel
* Merge pull request #87 from ros-simulation/remove_SDF_find_package_hydro
  Remove find_package(SDF) from CMakeLists.txt
* Standardized the way ROS nodes are initialized in gazebo plugins
* Remove find_package(SDF) from CMakeLists.txt
  It is sufficient to find gazebo, which will export the information
  about the SDFormat package.
* ROS Video Plugin for Gazebo - allows displaying an image stream in an OGRE texture inside gazebo. Also provides a fix for #85.
* Merge branch 'hydro-devel' of github.com:ros-simulation/gazebo_ros_pkgs into hydro-pcl-conversions
* Merge pull request #84 from ros-simulation/fix_prosilica_plugin
  patch a fix for prosilica plugin (startup race condition where rosnode\_ ...
* patch a fix for prosilica plugin (startup race condition where rosnode\_ might still be NULL).
* Merge pull request #82 from ros-simulation/hsu-groovy-devel
  add prosilica plugin (from pr2_gazebo_plugins)
* Added explanation of new dependency in gazebo_ros_pkgs
* switch Prosilica camera from type depth to regular camera (as depth data were not used).
* merging from hydro-devel
* migrating prosilica plugin from pr2_gazebo_plugins
* Merge branch 'groovy-devel' of https://github.com/ros-simulation/gazebo_ros_pkgs into groovy-devel
* Removed tbb because it was a temporary dependency for a Gazebo bug
* Revert "Added PCL to package.xml"
  This reverts commit 6b3b0b86178df29ab569def03954fec5f813a383.
* Revert "Added compiler conditionals for PCL 1.6 and 1.7 changes"
  This reverts commit a53077c84f63dbfcd61e2000c4968f4f34c506af.
  Conflicts:
  gazebo_plugins/CMakeLists.txt
  gazebo_plugins/src/gazebo_ros_depth_camera.cpp
  gazebo_plugins/src/pcl_conversions_compatibility.h
* Merge branch 'tranmission_parsing' into groovy-devel
* SDF.hh --> sdf.hh
* Merge pull request #78 from ros-simulation/merge_hydro_into_groovy
  Merge hydro into groovy
* Merge branch 'hydro-devel' into tranmission_parsing
* Merge branch 'hydro-devel' into merge_hydro_into_groovy
* Added PCL to package.xml
* Added note about pcl_conversions.h copied into this repo
* Small fixes to gazebo/hydro merge
* Merged hydro-devel branch in groovy-devel
* Added compiler conditionals for PCL 1.6 and 1.7 changes
* Merged hydro-devel
* Merged from Hydro-devel
* Contributors: Dave Coleman, John Hsu, Piyush Khandelwal, Steven Peters

2.1.5 (2013-07-18)
------------------
* changelogs for 2.1.5
* Include <sdf/sdf.hh> instead of <sdf/SDF.hh>
  The sdformat package recently changed the name of an sdf header
  file from SDF.hh to SDFImpl.hh; this change will use the lower-case
  header file which should work with old and new versions of sdformat
  or gazebo.
* Contributors: Steven Peters, Tully Foote

2.1.4 (2013-07-14)
------------------
* Bumped pkg version
* Updated changelogs
* Merge pull request #75 from ros-simulation/add_tbb_temp
  Add tbb temporarily to work around #74
* Contributors: Dave Coleman, Tully Foote

2.1.3 (2013-07-13)
------------------
* adding changelog 2.1.3
* temporarily add tbb as a work around for #74
* Contributors: Tully Foote

2.1.2 (2013-07-12)
------------------
* Added changelogs
* Merge pull request #73 from ros-simulation/pcl_upgrade_changes
  Fixed compatibility with new PCL 1.7.0 for Hydro
* Fixed compatibility with new PCL 1.7.0
* Merge pull request #71 from ros-simulation/enable_dyn_reconfig_camera
  Enable dyn reconfig camera
* Merge pull request #70 from ros-simulation/cmake_cleanup
  Cmake cleanup
* Tweak to make SDFConfig.cmake
* Merge pull request #69 from ros-simulation/dev
  Cleaned up gazebo_ros_paths_plugin
* Re-enabled dynamic reconfigure for camera utils - had been removed for Atlas
* Cleaned up CMakeLists.txt for all gazebo_ros_pkgs
* Removed SVN references
* Contributors: Dave Coleman, hsu

2.1.1 (2013-07-10)
------------------
* Merge branch 'hydro-devel' into dev
* Merge pull request #53 from ZdenekM/hydro-devel
  Minor improvement.
* Source code formatting.
* Merge branch 'hydro-devel' of https://github.com/ZdenekM/gazebo_ros_pkgs into hydro-devel
* Merge pull request #59 from ros-simulation/CMake_Tweak
  Added dependency to prevent missing msg header, cleaned up CMakeLists
* Merge pull request #63 from piyushk/patch-1
  install diff_drive and skid_steer plugins
* export diff drive and skid steer for other catkin packages
* install diff_drive and skid_steer plugins
* Merge branch 'CMake_Tweak' into dev
* Added dependency to prevent missing msg header, cleaned up CMakeLists
* Added ability to switch off publishing TF.
* Contributors: Dave Coleman, Piyush Khandelwal, ZdenekM

2.1.0 (2013-06-27)
------------------
* Merge branch 'hydro-devel' of github.com:osrf/gazebo_ros_pkgs into hydro-devel
* Merge pull request #51 from meyerj/fix_depth_and_openni_kinect_camera_plugin_segfaults
  Fix depth and openni kinect camera plugin segfaults
* gazebo_plugins: always use gazebo/ path prefix in include directives
* gazebo_plugins: call Advertise() directly after initialization has
  completed in gazebo_ros_openni_kinect and gazebo_ros_depth_camera
  plugins, as the sensor will never be activated otherwise
* Merge remote-tracking branch 'origin/hydro-devel' into robot_hw_sim
* Merge pull request #33 from meyerj/terminate_service_thread_fix
  another fix for terminating the service_thread\_ in PubQueue.h
* Merge branch 'hydro-devel' of https://github.com/osrf/gazebo_ros_pkgs into terminate_service_thread_fix
  Conflicts:
  gazebo_plugins/include/gazebo_plugins/PubQueue.h
* Merge pull request #41 from ZdenekM/hydro-devel
  Added skid steering plugin (modified diff drive plugin).
* Merge pull request #35 from meyerj/fix_include_directory_installation_target
  Header files of packages gazebo_ros and gazebo_plugins are installed to the wrong location
* Rotation fixed.
* Skid steering drive plugin.
* Merge branch 'hydro-devel' of github.com:osrf/gazebo_ros_pkgs into hydro-devel
* Merge pull request #31 from meyerj/fix_depth_and_openni_kinect_camera_plugin_segfaults
  Segfault using the gazebo_ros_openni_kinect plugin
* Merge pull request #30 from osrf/deprecated-groovy
  fix for terminating the service_thread\_ in PubQueue.h
* gazebo_plugins: added missing initialization of GazeboRosDepthCamera::advertised\_
* gazebo_plugins: fixed depth and openni kinect camera plugin segfaults
* gazebo_plugins: terminate the service thread properly on destruction of a PubMutliQueue object without shuting down ros
* gazebo_plugins/gazebo_ros: fixed install directories for include files and gazebo scripts
* fix for terminating the service_thread\_ in PubQueue.h
* Merge pull request #27 from piyushk/add-diff-drive-plugin
  added differential drive plugin to gazebo plugins
* added differential drive plugin to gazebo plugins
* Contributors: Dave Coleman, Fadri Furrer, Johannes Meyer, Piyush Khandelwal, ZdenekM

2.0.2 (2013-06-20)
------------------
* Added Gazebo dependency
* Contributors: Dave Coleman

2.0.1 (2013-06-19)
------------------
* Incremented version to 2.0.1
* Merge pull request #18 from osrf/check_camera_util_is_init
  Check camera util is initialized before publishing - fix from Atlas
* Fixed circular dependency, removed deprecated pkgs since its a stand alone pkg
* Check camera util is initialized before publishing - fix from Atlas
* Contributors: Dave Coleman

2.0.0 (2013-06-18)
------------------
* Changed version to 2.0.0 based on gazebo_simulator being 1.0.0
* Updated package.xml files for ros.org documentation purposes
* Merge pull request #15 from osrf/topics_services
  Revamped Gazebo Services
* Combined updateSDFModelPose and updateSDFName, added ability to spawn SDFs from model database, updates SDF version to lastest in parts of code, updated the tests
* Created tests for various spawning methods
* Added debug info to shutdown
* Fixed gazebo includes to be in <gazebo/...> format
* Merge pull request #11 from osrf/plugin_updates
  Merged Atlas ROS Plugins
* Cleaned up file, addded debug info
* Merge branch 'groovy-devel' into plugin_updates
* Merge pull request #10 from osrf/bug-curved-laser
  John agrees that this should be merged, this was after we forked from simulator_gazebo. Thanks!
* Merged changes from Atlas ROS plugins, cleaned up headers
* Merged changes from Atlas ROS plugins, cleaned up headers
* fix curved laser issue
* Combining Atlas code with old gazebo_plugins
* Combining Atlas code with old gazebo_plugins
* Merge pull request #8 from osrf/code_cleanup
  Code cleanup
* Small fixes per ffurrer's code review
* Merge pull request #6 from fmder/tf-prefix
  Added the robot namespace to the tf prefix.
* Added the robot namespace to the tf prefix.
  The tf_prefix param is published under the robot namespace and not the
  robotnamespace/camera node which makes it non-local we have to use the
  robot namespace to get it otherwise it is empty.
* findreplace ConnectWorldUpdateStart ConnectWorldUpdateBegin
* Fixed deprecated function calls in gazebo_plugins
* Deprecated warnings fixes
* Removed the two plugin tests that are deprecated
* Removed abandoned plugin tests
* All packages building in Groovy/Catkin
* Imported from bitbucket.org
* Contributors: Dave Coleman, FIXED-TERM Hausman Karol (CR/RTC1.1-NA), François-Michel De Rainville, hsu
