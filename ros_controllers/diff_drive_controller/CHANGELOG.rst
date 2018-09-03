^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package diff_drive_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2018-01-10)
------------------
* increased tolerance in diff drive test from 0.03 to 0.04 in orientation
* removed changelogs
* consistent package versions
* Contributors: Hilario Tome

0.3.2 (2018-01-12)
------------------

0.3.3 (2018-03-14)
------------------
* Merge branch 'dub-port/warnings' into 'erbium-devel'
  Dub port/warnings
  See merge request control/ros_controllers!25
* Merge branch 'dub-port/per_wheel_multiplier' into 'erbium-devel'
  Dub port/per wheel multiplier
  See merge request control/ros_controllers!26
* duplicate gtest diff_drive_multipliers
  duplicate to test left/right wheel multipliers
* per wheel radius multiplier
  Conflicts:
  diff_drive_controller/src/diff_drive_controller.cpp
* fix warning
* separate include_directories
  ${catkin_INCLUDE_DIRS} as SYSTEM to avoid unrelated
  compilation warning spamming
* Contributors: Hilario Tome, Jeremie Deray

0.3.4 (2018-03-28)
------------------
* Merge branch 'gtest/fixes' into 'erbium-devel'
  gtest waitForCmdVelOutMsgs
  See merge request control/ros_controllers!30
* fix xacro macro warning
* skidsteerbot publishes its own clock
* Merge branch 'dynamic_reconf' into 'erbium-devel'
  Dynamic reconf
  See merge request control/ros_controllers!28
* skid_steer_common use_sim_time
* fix testTurn
* gtest waitForCmdVelOutMsgs
* Merge branch 'gtest/fixes' into 'erbium-devel'
  Gtest/fixes
  See merge request control/ros_controllers!27
* diffbot simulation clock
* gtest dynamic_reconf
* add dynamic_reconf to diff_drive_controller
* fix fail_test
* fix multiple_cmd_vel_publishers_test
* fix pub_cmd_vel_out_test
* fix default_cmd_vel_out_test
* fix diff_drive_test
* fix default_odom_frame_test
* fixe odom_frame_test
  avoid testing on a default constructed message.
* gtest add func waitForController & waitForOdomMsgs
* Contributors: Hilario Tome, Jeremie Deray, Victor Lopez

0.3.5 (2018-04-05)
------------------

0.3.6 (2018-05-02)
------------------
* Merge branch 'diffdrive_small_fix' into 'erbium-devel'
  call initRT rather than writeFromNonRT on init
  See merge request control/ros_controllers!33
* call initRT rather than writeFromNonRT on init
* Contributors: Hilario Tome, Jeremie Deray

0.3.7 (2018-05-07)
------------------

0.3.8 (2018-05-08)
------------------

0.3.9 (2018-05-29)
------------------
* Merge branch 'dyn_reconf_client_test' into 'erbium-devel'
  add dyn_reconf client test
  See merge request control/ros_controllers!32
* add dyn_reconf client test
* Contributors: Hilario Tome, Jeremie Deray

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
* Merge pull request #287 from bmagyar/diff_drive_multiple_publishers_check_indigo_port
  add check for multiple publishers on cmd_vel
* Add test for allow_multiple_cmd_vel_publishers param
* add check for multiple publishers on cmd_vel
* Merge pull request #286 from bmagyar/diff_drive_odom_frame_parameter_indigo_port
  Diff drive odom frame parameter indigo port
* Reduced pedantry, redundancy.
* Added tests for the odom_frame_id parameter.
* Parameterized diff_drive_controller's odom_frame_id
* Merge pull request #285 from bmagyar/publish_exec_velocity_if_publish_cmd_indigo_port
  Publish exec velocity if publish cmd indigo port
* do not instantiate cmd_vel_out pub if !publish_cmd\_
* add tests cmd_vel_out
* Publish executed velocity if publish_cmd
* Merge pull request #278 from ros-controls/sphere_drive_kinetic
  Add support for spherical wheels in diff_drive_controller
* refactor to remove code duplication
* fixup pointer type for new convention
* touchups from review of #259
* Add square wheel xacro and modify tests to allow spherical wheels (but not square)
* Allow diff_drive_controller to use spheres as well as cylinders for wheel collision geometry. Cylinders are not well behaved on Gazebo/ODE heightfields, using spheres works around the issue.
* Contributors: Bence Magyar, Eric Tappan, Gennaro Raiola, Jeremie Deray, Karsten Knese, Tully Foote, mallanmba, tappan-at-git

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
* Merge pull request #258 from bmagyar/diff_drv_export_include_dirs
  Add exporting include dirs
* Add exporting include dirs
* Contributors: Bence Magyar

0.12.0 (2017-02-15)
-------------------
* Update changelogs
* Merge pull request #242 from bmagyar/update_package_xmls
  Update package xmls
* Fix most catkin lint issues
* Change for format2
* Add Enrique and Bence to maintainers
* Merge pull request #246 from bmagyar/add_urdf_compatibility
  Add urdf compatibility header
* Add urdf compatibility header
* Merge pull request #239 from ros-controls/fix-xacro-warnings
  Fix xacro-related warnings
* Add --inorder to xacro calls
* Add missing xacro tags
* Use xacro instead of xacro.py
* Merge pull request #241 from bmagyar/disable-travis-failing-test
  Disable angular jerk limit test
* Disable angular jerk limit test
* Merge branch 'kinetic-devel' into F_enable_part_traj_kinetic
* Merge pull request #235 from bmagyar/unboost-urdf-fix
  Replace boost::shared_ptr<urdf::XY> with urdf::XYConstSharedPtr when exists
* Replace boost::shared_ptr<urdf::XY> with urdf::XYConstSharedPtr when exists
* Contributors: Bence Magyar, Enrique Fernández Perdomo, beatrizleon

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
* Merge pull request #183 from efernandez/sync_with_pal_hydro-devel
  Sync with hydro-devel branch in PAL Robotics fork
* Limit jerk
* Add param velocity_rolling_window_size
* Minor fixes
  1. Coding style
  2. Tolerance to fall-back to Runge-Kutta 2 integration
  3. Remove unused variables
* Fix forward test
  Fix the following bugs in the testForward test:
  1. Check traveled distance in XY plane
  2. Use expected speed variable on test check
* Add test for NaN
* Add test for bad URDF
  This unit test exercises a controller load failure caused by
  a wrong wheel geometry. The controller requires that wheels be
  modeled by cylinders, while the bad URDF uses spheres.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Enrique Fernandez, Paul Mathieu

0.9.2 (2015-05-04)
------------------
* Update changelogs.
* Merge pull request #169 from bmagyar/separation_and_diameter_params
  [diff_drive_controller] Add wheel_separation and wheel_radius parameters
* remove diff_drive_bad_urdf.test
  The 'bad' URDF is not that bad any more since the wheel radius can be set via parameter for non-cylindrical wheels.
  However, it used outdated joint names that would cause controller creation to fail even with the wheel_radius parameter set.
  Since diff_drive_wrong.test already checks that case, and diff_drive_radius_param_fail.test explicitly checks for behavior with missing wheel_radius, diff_drive_bad_urdf.test is obsolete
* style and whitespace fixes
* move short-circuit of URDF lookup into setOdomParamsFromUrdf
* add test for wheel_separation parameter
  this doesn't technically test whether the separation is set correctly, but observing the controller's info output shows that it is
* add test that checks wheel_radius parameter functionality
* add test that checks that controller initialization fails when wheels are not cylinders and no wheel_radius parameter is given
* add a sphere-wheeled version of diffbot to test the new wheel_radius parameter
* allow the wheel separation and radius to be set from different sources
  i.e. one can be set from the URDF, the other from a parameter
* If wheel separation and wheel diameter is specified, don't look them up from urdf
* Contributors: Adolfo Rodriguez Tsouroukdissian, Bence Magyar, Nils Berg

0.9.1 (2014-11-03)
------------------
* Update changelogs
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.0 (2014-10-31)
------------------
* Update changelogs
* Merge pull request #138 from bmagyar/diff_drive_skid_steer
  diff drive skid steer
* Merge pull request #137 from bmagyar/diff_drive_remove_angles
  diff drive remove angles
* Merge pull request #136 from bmagyar/diff_drive_open_loop_odom
  Diff drive open loop odom
* Fix test xacros
* fixes missing std_srvs test dependency
* simplifies error checks
* adds no wheels test
* adds skid steer bot test
* adds support for multiple wheel joints per side
* removes angle normalization
* cosmetic changes
* adds odometry init to reset timestamp and accs
* adds open loop test
* Cosmetic refactoring
* adds open loop odometry
* Merge pull request #120 from pal-robotics/diff_drive_param_enable_odom_tf
  Add default-true parameter "~enable_odom_tf"
* Add default-true parameter "~enable_odom_tf"
* Merge pull request #113 from bulwahn/indigo-devel
  addressing test dependencies with -DCATKIN_ENABLE_TESTING=0
* diff_drive_controller: add realtime_tools dependency
  When executing 'catkin_make -DCATKIN_ENABLE_TESTING=0', this error occurs:
  In file included from [...]/diff_drive_controller/src/diff_drive_controller.cpp:45:0:
  [...]/diff_drive_controller/include/diff_drive_controller/diff_drive_controller.h:46:44: fatal error: realtime_tools/realtime_buffer.h: No such file or directory
  #include <realtime_tools/realtime_buffer.h>
  ^
  compilation terminated.
  make[2]: *** [ros_controllers/diff_drive_controller/CMakeFiles/diff_drive_controller.dir/src/diff_drive_controller.cpp.o] Error 1
  Obviously, the realtime_tools dependency was missing in the CMakeLists.txt file.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Bence Magyar, Lukas Bulwahn, efernandez, enriquefernandez

0.8.1 (2014-07-11)
------------------
* Update chegelogs
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.8.0 (2014-05-12)
------------------
* Updated changelogs
* Add base_frame_id param (defaults to base_link)
  The nav_msgs/Odometry message specifies the child_frame_id field,
  which was previously not set.
  This commit creates a parameter to replace the previously hard-coded
  value of the child_frame_id of the published tf frame, and uses it
  in the odom message as well.
* Contributors: Dave Coleman, enriquefernandez

0.7.2 (2014-04-01)
------------------
* Prepare 0.7.2
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.7.1 (2014-03-31)
------------------
* Prepare 0.7.1
* Merge pull request #85 from bmagyar/fix_diff_drive_release
  Changed test-depend to build-depend for release jobs.
* Changed test-depend to build-depend for release jobs.
* 0.7.0
* Create changelog files for new packages.
* Create README.md
* Merge pull request #78 from po1/patch-1
  diff_drive: bump version to 0.6.0
* diff_drive: bump version to 0.6.0
  To match the rest of `ros_controllers`
* Merge pull request #77 from pal-robotics/diff-drive-controller
  Controller for differential drive wheel base
* Added documentation about assumptions + comment aesthetics
* diff_drive: add unit test for bad urdf
  This unit test exercises a controller load failure caused by
  a wrong wheel geometry. The controller requires that wheels be
  modeled by cylinders, while the bad URDF uses spheres.
* diff_drive: add test for controller load failure
  This test exercises a controller load failure due to a misnamed
  wheel link name in the controller configuration file.
* diff_drive: use backslashes in doxygen documentation
* diff_drive: fix indentation of diffbot.xacro
* diff_drive: refactor integrate* methods in odometry
  refs #30
* diff_drive: resize the TF odom publisher at init
  This is to avoid push_back mem allocations inside of the real-time loop
* diff_drive: set version to 0.5.4
* diff_drive: add install rule for the plugin xml
* adds angular velocity/acceleration limits tests
* fixes default min velocity/acceleration params
* diff_drive: fix comments
* diff_drive: add unit test for parameter multipliers
* diff_drive: fix dependencies
  Too few in CMakeLists.txt, too many in package.xml
* diff_drive: fix nasty crash on some platforms (OSX)
  This one is nasty. Because we _manually\_ load the controller at runtime,
  we have to build it with special flags to export the run-time type
  information (rtti) needed for stuff like dynamic_cast. But we don't, and
  here it's simpler to just use a static_cast.
  See here: http://gcc.gnu.org/faq.html#dso
  A dynamic_cast without a check is stupid anyway. (_what could possibly
  go wrong?..._)
* diff_drive: change the limiter policy to open-loop
  Also prevent a potential problem with uninitialized command structure
* diff_drive: use wheel separation and radius multipliers for the odometry
* diff_drive: add unit test for cmd_vel_timeout
* diff_drive: update unit tests
* diff_drive: rename cmd_vel_old_threshold to cmd_vel_timeout
  The default is now 0.5s instead of 1.0s
* diff_drive: add missing dependency to controller_interface
* diff_drive: cosmetic fix
  For a comment, in a launch file, for a unit test.
  Quite the commit, right?
* diff_drive: small fix in SpeedLimiter doc
* diff_drive: add unit test for velocity and acceleration limits
* diff_drive: fix acceleration limit
  The dt was wrong.
* diff_drive: add parameter file for limits in tests
* diff_drive: fix indentation of yaml file
* diff_drive: factorize (future) common launch files in tests
* diff_drive: factorize (future) common code in tests
* diff_drive: fix copyright notice
* diff_drive: format SpeedLimiter constructor to avoid long lines
* diff_drive: change 'Willow Garage' to 'PAL Robotics' in the license
* diff_drive: remove struct for linear and angular SpeedLimiter objects
* diff_drive: add speed limiter for velocity and acceleration limits
* diff_drive: clean and comments code that sets the wheels velocities
* diff_drive: fix unit test
* creates implementation for odometry
* adds doxygen doc for private methods
* adds doxygen doc for public methods
* creates header for diff_drive_controller
* Integrate odometry position even with very small intervals
* doesn't update odometry when dt < 0.0001 (note that befor the *_wheel_old_pos\_ was changed every iteration)
* fixes left/right wheel typo in a logging msg
* diff_drive: add catkin includes to unit tests
* added topics to controller_nh (instead of root_nh), to be consistent with all the other controllers
* remove ROS logging messages from RT-safe methods
* added brake method to avoid using the stopping method, which would break RT constraints
* added cmd_vel_old_threshold param and logic to stop when the cmd_vel command is too old
* fix member constructors ordering
* put static functions out of the class
* take wheel radius from wheel link cylinder radius + cleanup
* added wheel separation and radius params
* fix heading normalization, using ROS angles lib + sort members constructors
* diff_drive: fixed linking to urdf
* diff_drive: fix unit test (bullet types)
* diff_drive: odometry refactoring
* Update pluginlib macros to newer ones
* Refactored code and removed / from cmd topic name in test.
* Removed absolute path from robot_description
* diff_drive: update license in header file
* diff_drive: use add_rostest_gtest() in CMakeLists.txt
* Use <test_depend> and put some order in package.xml
* Catkinized package
* Removed print
* Removed unused codes and added separate tolerances for position and orientation.
* Changed checks on twist fields. Not interested in the difference but the final value of those fields. Checking for those.
* Changed ASSERT_TRUE statements to EXPECT_LT and EXPECT_GT expressions.
* Test halfway fixed.
* Added turn test.
* Fixed test.
* Broken test but on the right way
* Publishing odom even when there were no changes.
* Trying to refactor everything so that it works with both REEM and the test robot.
  Added urdf link iteration and changed robot format to xacro + added params.
* Removed print.
* Moved odom and cmd_vel topics to global namespace.
* Changed robot urdf to one from ROS tutorials.
  Working on setting it up to work.
* Adding test infrastructure to controller with dummy robot + test node skeleton.
* Updated manifest with unlisted dependencies + new deps for testing.
* Added parsing of covariance params.
* Fixed comments.
* Made a few variables local.
* Removed unnecessary linear\_ and angular\_ fields.
* Removed copy constructor, assignment op, operator= since they were the same as the default.
* Moved getLeafNamespace function inline.
* Moved odometry computation into odometry class.
* Removed inlines and changed prefix _ to postfix _ for member fields.
* Added publish rate param + handling.
* Fixed tf publishing error. Separated and minimized codes which are using locks.
* Moved ROS_INFO-s to ROS_DEBUG or removed. Added typedef for iterator type.
* Moved static member to field
* Cleaning up/refactoring code
* Added tf publishing. The behaviour is not as expected, have to investigate.
* Added odometry publishing to controller. Needs test with navigation.
* Added Odometry class refactored from old code. Updated manifest for dependencies.
* Fixed speed problems.
* Added subscriber + real time stuff for cmd_vel subscription. Robot moves with joystick.
* Added parameter and simple urdf parsing and realtime odometry publisher.
* Added odom message type + fixed loading problems of plugin.
* Added package for diff_drive_controller.
  Had to create a modified version of CMakeLists.txt for qt creator to work, will remove later.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Bence Magyar, Paul Mathieu, enriquefernandez

0.6.0 (2014-02-05)
------------------

0.5.4 (2013-09-30)
------------------

0.5.3 (2013-09-04)
------------------

0.5.2 (2013-08-06)
------------------

0.5.1 (2013-07-19)
------------------

0.5.0 (2013-07-16)
------------------

0.4.0 (2013-06-26)
------------------
