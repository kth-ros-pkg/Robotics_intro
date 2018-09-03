^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_trajectory_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2018-01-10)
------------------
* removed changelogs
* consistent package versions
* Merge pull request #304 from mmoerdijk/enable_acceleration_forwarding
  Enable forwarding of the acceleration values from the trajectory
* Changend the implementation of joint_trajectory_controller to enable the forwarding of the acceleration values from the trajectory
* Contributors: Bence Magyar, Hilario Tome, mmj

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
* Merge branch 'feedback_null_pointer' into 'erbium-devel'
  added safeguard for feedback null pointer
  See merge request control/ros_controllers!36
* Added protection for rt_active_goal\_ in callbacks
* removed cmakeList.user
* added safeguard for feedback null pointer
* Contributors: Hilario Tome

0.3.10 (2018-06-15)
-------------------

0.3.11 (2018-06-19)
-------------------

0.13.1 (2017-11-06)
-------------------
* Update changelogs
* Merge pull request #296 from Zabot/kinetic-spline-trajectory-controller
  Rebased #141: Add JointTrajectoryController specification for SplineJointInterface
* Linted pos_vel joint_trajectory_controllers
* Added posvel joint_trajectory_controller
  Added a simple posvel joint_trajectory_controller that forwards
  the desired state at the current point in time of the trajectory
  to the joint.
* Add support for an joint interfaces are not inherited from JointHandle.
  Add JointTrajectoryController specification for SplineJointInterface.
* Contributors: Bence Magyar, Gennaro Raiola, Igorec, Zach Anderson

0.13.0 (2017-08-10)
-------------------
* Update changelogs
* Merge pull request #273 from bponsler/kinetic-expose-verbose
  Exposed the joint trajectory controller verbose setting as a parameter.
* Merge pull request #276 from graiola/kinetic-devel
  Address issue #263, joint_trajectory_controller - wraparoundOffset
* Make rqt_plot optional
* Added tests for issue #275
* Address Issue  #275 for kinetic
* Address issue #263, joint_trajectory_controller - wraparoundOffset
* Added warning to indicate that the verbose flag is enabled
* Merge pull request #271 from miguelprada/empty_trajectory_action_fix
  Set hold trajectory goal handle when empty trajectory action is received.
* Set hold trajectory goal handle when empty trajectory received through action.
  Previously, an empty trajectory received through the action interface would
  set hold trajectory and accept the action goal, but the action would never be
  terminated, leaving clients hanging.
* Contributors: Bence Magyar, Miguel Prada, bponsler, gennaro

0.12.3 (2017-04-23)
-------------------
* Update changelogs
* Contributors: Bence Magyar

0.12.2 (2017-04-21)
-------------------
* Update changelogs
* Merge pull request #265 from bmagyar/remove_rqt_plot_test_depend
  Remove rqt_plot test_depend
* Remove rqt_plot test_depend & make plots optional
* Contributors: Bence Magyar, Mathias Lüdtke

0.12.1 (2017-03-08)
-------------------
* Update changelogs
* Contributors: Bence Magyar

0.12.0 (2017-02-15)
-------------------
* Update changelogs
* Merge pull request #242 from bmagyar/update_package_xmls
  Update package xmls
* Fix missing controller_manager include
* Ordered dependencies & cleanup
* Change for format2
* Add Enrique and Bence to maintainers
* Merge pull request #239 from ros-controls/fix-xacro-warnings
  Fix xacro-related warnings
* Merge pull request #236 from bmagyar/joint_traj_old_traj_crash
  Add test that sends trajectory entirely in past
* Add test that sends trajectory entirely in past
* Use xacro instead of xacro.py
* Merge pull request #237 from bmagyar/unboost-urdf-last-bit
  urdf::Model typedefs had to be added to a different repo first
* urdf::Model typedefs had to be added to a different repo first
* Merge pull request #226 from shadow-robot/F_enable_part_traj_kinetic
  jtc: Enable sending trajectories with a partial set of joints
* Updated copyright info
* jtc: Enable sending trajectories with a partial set of joints
* Merge pull request #228 from miguelprada/velocity_iface_tests
  Add tests for velocity_controllers::JointTrajectoryController
* Merge branch 'kinetic-devel' into F_enable_part_traj_kinetic
* Merge pull request #235 from bmagyar/unboost-urdf-fix
  Replace boost::shared_ptr<urdf::XY> with urdf::XYConstSharedPtr when exists
* Replace boost::shared_ptr<urdf::XY> with urdf::XYConstSharedPtr when exists
* Infrastructure for testing the velocity_controllers::JointTrajectoryController.
* jtc: Enable sending trajectories with a partial set of joints
* Contributors: Beatriz Leon, Bence Magyar, Enrique Fernández Perdomo, Miguel Prada, beatrizleon

0.11.2 (2016-08-16)
-------------------
* Update changelogs
* Contributors: Bence Magyar

0.11.1 (2016-05-23)
-------------------
* Update changelogs
* Merge pull request #219 from bmagyar/joint_traj_action_feedback
  Write feedback for the RealtimeServerGoalHandle to publish -- rebased
* Write feedback for the RealtimeServerGoalHandle to publish on the non-realtime thread.
* Contributors: Bence Magyar, Miguel Prada

0.11.0 (2016-05-03)
-------------------
* Update changelogs
* Contributors: Bence Magyar

0.10.0 (2015-11-20)
-------------------
* Update changelogs
* Merge pull request #189 from ros-controls/rqt-jtc-improvements
  [rqt_joint_trajectory_controller] Many small improvements
* jtc: Add joint limits spec to rrbot test robot
* Address -Wunused-parameter warnings
* Merge pull request #186 from ros-controls/w-unused-parameter
  Address -Wunused-parameter warnings
* Address -Wunused-parameter warnings
* Merge pull request #170 from ipa-fxm/fix_semantic_zero_hwi_adapter
  [Indigo] Reset to semantic zero in HardwareInterfaceAdapter for PositionJointInterface
* reset to semantic zero in HardwareInterfaceAdapter for PositionJointInterface
* Contributors: Adolfo Rodriguez Tsouroukdissian, ipa-fxm

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
* Merge pull request #145 from pal-robotics/check-waypoint-timing
  Check waypoint timing
* Add missing dependency to tests target
* Check that waypoint times are strictly increasing
  A precondition for all trajectories executed by the
  joint_trajectory_controller is that waypoints must have strictly
  increasing reach times. This changeset validates the precondition and
  rejects commands that don't satisfy it.
* Merge pull request #133 from pal-robotics/catkin-lint-fixes
  Buildsystem fixes suggested by catkin_lint
* Buildsystem fixes suggested by catkin_lint
* Merge pull request #123 from pal-robotics/jtc-install-all-headers
  Add trajectory_interface headers to install target
* Merge pull request #116 from ipa-fxm/feature/velocity_controllers/JointTrajectoryController_indigo
  feature/velocity_controllers/joint_trajectory_controller rebased to indigo-devel
* Add trajectory_interface headers to install target
* add velocity interface for joint_trajectory_controller in separate feature branch - feature provided by @davetcoleman
* Merge pull request #113 from bulwahn/indigo-devel
  addressing test dependencies with -DCATKIN_ENABLE_TESTING=0
* joint_trajectory_controller: make rostest in CMakeLists optional (ros/rosdistro#3010)
* Contributors: Adolfo Rodriguez Tsouroukdissian, Lukas Bulwahn, ipa-fxm

0.8.1 (2014-07-11)
------------------
* Update chegelogs
* Merge pull request #97 from jbohren-forks/critical-cmake
  joint_trajectory_controller: Critical targets declared before calling catkin_package
* joint_trajectory_controller: Critical targets declared before calling catkin_package
* Merge pull request #95 from bulwahn/indigo-devel
  check for CATKIN_ENABLE_TESTING
* check for CATKIN_ENABLE_TESTING
* Contributors: Adolfo Rodriguez Tsouroukdissian, Jonathan Bohren, Lukas Bulwahn

0.8.0 (2014-05-12)
------------------
* Updated changelogs
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
* Merge pull request #82 from Igorec/joint_trajectory_controller
  Added support to JointTrajectoryController for an joint interfaces are not inherited from JointHandle.
* Add support for an joint interfaces are not inherited from JointHandle.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Igorec

0.6.0 (2014-02-05)
------------------
* Updated changelogs
* Merge pull request #72 from pal-robotics/minor-maintenance
  Minor maintenance
* Merge pull request #74 from pal-robotics/stop_traj_duration_default
  Default stop_trajectory_duration to zero. Refs #73
* Default stop_trajectory_duration to zero. Refs #73
* Better logs when dropping traj points. Refs #68.
* Fix class member reorder warning in constructor.
* Add missing headers to target files.
* Merge pull request #71 from pal-robotics/issue-70
  Action interface rejects empty goals. Fixes #70.
* Action interface rejects empty goals. Fixes #70.
* Merge pull request #69 from pal-robotics/issue-60
  Fix issue #60
* Reorder how time and traj data are updated.
  In the update method, fetching the currently executed trajectory should be done
  before updating the time data to prevent a potential scenario in which there
  is no trajectory defined for the current control cycle.
* Work tolerance checking methods.
  Until now we used the currently active goal handle for performing tolerance
  checks. Using the goal handle stored in segments is more robust to unexpected
  goal updates by the non-rt thread.
* Refactor how the currrent trajectory is stored.
  - Handle concurrency in the current trajectory between rt and non-rt threads
  using the simpler RealtimeBox instead of the RealtimeBuffer, because our
  usecase does not fit well the non-rt->writes / rt->reads semantics.
  - As a consequence we no longer need to store the msg_trajectory member, but
  only the hold_trajectory, which must still be preallocated.
* Merge pull request #67 from pal-robotics/issue-65
  Honor unspecified vel/acc in ROS message. Fix #65.
* Honor unspecified vel/acc in ROS message. Fix #65.
* Merge pull request #61 from ros-controls/joint_trajectory_tweaks
  Joint trajectory improved debugging
* Fixes per Adolfo
* Added verbose flag
* Fixing realtime issues
* Merge branch 'hydro-devel' into joint_trajectory_tweaks
* Tweaked error messages
* Added more debug info
* Merge branch 'joint_trajectory_tweaks' into development
* Merge branch 'hydro-devel' into development
* Fix for microsecond delay that caused header time=0 (now) to start too late
* Reworded debug message
* Merge branch 'hydro-devel' of https://github.com/willowgarage/ros_controllers into hydro-devel
* Image update.
* Update README.md
  Factor out user documentation to the ROS wiki.
* Merge branch 'hydro-devel' of https://github.com/willowgarage/ros_controllers into hydro-devel
* Rename hold_trajectory_duration
  - hold_trajectory_duration -> stop_trajectory_duration for more clarity.
  - During Hydro, hold_trajectory_duration will still work, giving a deprecation
  warning.
* Add basic description in package.xml.
* Add images used in the ROS wiki doc.
* Added better debug info
* Throttled debug output
* Added more debug and error information
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.4 (2013-09-30)
------------------
* Updated changelogs
* Merge pull request #50 from ros-controls/plugin_xml_install
  Added install rules for plugin.xml
* Added install rules for plugin.xml
* Remove PID sign flip.
  This is now done in the state error computation.
* Merge pull request #45 from ros-controls/effort_fixes
  Added check for ~/robot_description and fixed hardware interface abstraction bug
* Flip state error sign.
* Merge branch 'hydro-devel' of https://github.com/willowgarage/ros_controllers into hydro-devel
* PID sign was wrong
* Added check for ~/robot_description and fixed hardware interface abstraction bug
* Update README.md
* Create README.md
* Fix license header string for some files.
* Less verbose init logging.
  Statement detailing controller joint count, as well as segment and hardware
  interface types moved from INFO to DEBUG severity.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.3 (2013-09-04)
------------------
* Update changelogs for 0.5.3.
* Make cmake_modules dependency explicit.
* Merge pull request #38 from pal-robotics/joint_trajectory_controller_hydro
  Joint trajectory controller
* Fix remaining Hydro build and test issues.
* Comment xacro dependency as it breaks the build.
  TODO: Figure out what's going on here?.
* Change return type of permutation() function.
  Problem: permutation return type was vector<T>::size_type, which is:
  - 32bits wide in 32bits Unix, which is the size of both unsigned int
  and unsigned long int.
  - 64bits wide in 64bits Unix, which is the size of unsigned long int,
  but not unsigned int (the latter being 32bits wide).
  I was doing at some points vector<unsigned int> = permutation(...),
  which triggered a compile error in 64bit systems. Due to this, I'm
  changing the return type of the function to be vector<unsigned int>,
  and live with the implicit narrowing conversions that will take
  place inside the implementation, since we won't be storing vectors
  large enough to overflow an unsigned int.
* Merge pull request #1 from davetcoleman/joint_trajectory_controller_hydro
  Small CMake tweaks for catkin
* Use correct return type when calling permutation()
  - Fixes a 32-64bit issue.
* Small CMake tweaks for catkin
* Hydro compatibility patches.
* Implement hold traj without direction reversals.
  - Implement hold trajectory (executed whenever a trajectory is cancelled, or the
  controller is started) without resulting in direction reversals.
  - Implementation assumes that a segment going from (pos, vel) to (pos,-vel) is
  symmetric and has a zero-velocity point in the middle.
  - Reduce default hold trajectory duration to 0.5s.
* Make hold trajectory duration configurable.
* Silence gcc warning on mismatching enumeral types.
* Complete test suite for controller.
* Allow to tune robot hardware simulator fidelity.
  - Add a smoothing topic, that allows to set an exponential smoothing factor,
  where 0 means perfect control and 1 means no control at all (don't move).
* Restore use of INVALID_JOINTS error code.
  - When rejecting a goal for invalid joints reasons, use INVALID_JOINT, and not
  INVALID_GOAL.
* Trivial log message fix.
* Fix bug where goal handle was not being reset.
  - When an action goal failed due to path constraint violations, the currently
  active goal was not being reset.
* Extend controller rostest.
  - Pretty much all of the controller code is exercised, except for tolerance
  checking.
* Remove unused variable.
* Handle singularity at pi when computing wraparound.
  - angles::shortest_angular_distance() has a singularity at pi, that can yield
  unexpected results when computing wraparound values. This is now taken into
  account.
  - Update unit tests.
* Log exception string when traj update fails.
* First version of catkin-spcific scripts. Untested.
* Add minimal controller doc.
* Initial controller rostest.
  - Currently only exercises topic interface.
* Trivial cosmetic fixes.
* HardwareInterfaceAdapter now also sets commands.
* Add return value to updateTrajectoryCommand method
  - Leverage this to discard invalid trajectory messages (eg. too old) and not
  preempt active goals when such invalid messages arrive.
* Better logging messages
  - Use NAMED log statements, to differentiate individual controllers.
  - More informative info message on controller init: No. of joints, hw interface
  and segment type are reported.
* Templatize controller on HW iface & segment types.
  - JointTrajectoryController is now a template class that depends on the
  Hardware interface type, as well as on the trajectory segment type. This
  allows to reuse the code across multiple combinations of these two parameters.
  We currently offer plugins for quintic spline segments and position or
  effort hardware interfaces.
  - Create a HardwareInterface Adapter class, that converts desired+error states
  from the trajectory sampler (pos, vel, acc) to commands of a given hardware
  interface. Currently implemented adapters for position and effort interfaces.
* Unit test tolerances-related functionality.
* Move tolerances to separate header.
  - Fetch default tolerances from ros param server in controller.
  - Document tolerance-realted functions.
* Implement monitoring ROS API.
  - Add the "query_state" ROS service.
  - Add the "state" topic, whose publish rate is configurable.
  - Add checks that either error-out or do a no-op when the ROS API is excercised
  in a stopped state. This is important because the ROS API is available when
  the controller is initialized, but not yet running.
  - Label realtime and non-realtime methods on public class API.
* Unit test trajectory init with different time base.
  - Test initialization from ROS message when current trajectory and message
  are represented in different time bases.
  - Fix bug in implementation exposed by the test :)
* Protect shared time data behind a realtime buffer.
  Note: We're using the realtime_tools::RealtimeBuffer the wrong way around,
  ie. we're writing from a realtime thread, and reading from a non-realtime
  thread. The ideal solution would be to use a lock-free data structure.
* Drop usage of system clock in controller loop.
  Internal trajectory representation is no longer parameterized on the system
  clock, but on a monotonically increasing variable representing controller uptime
  (the base is irrelevant, the important thing is that it's monotonically
  increasing).
  This solves the problem that if the system time changes during controller
  operation, the result is a potentially dangerous discontinuity in the joint
  commands. It's true that one should not perform abrupt time changes during
  operation, and only do very small corrections (eg. NTP slew), but it stands
  to reason (and to the principle of least surprise) that already queued
  commands should not be affected by an external influence like a time change.
  When a new trajectory command arrives, it will be immediately transformed to the
  monotonic reference. In this way, system time changes affect only new commands
  arriving _after\_ the change, and not already queued ones.
* Make starting() method realtime-safe.
* init method is now part of the Segment public API.
* Add missing \endcode in doc.
* Scalar type fully templated accross the board.
  To make this happen, and good for other reasons as well: separate State in
  QuinticSplineSegment to a separate file.
* Expose Scalar type as segment typedef.
* Unit-test trajectory state tolerance checking.
* Rename constraints -> tolerances in code.
* Add goal handle value checking to init tests.
* Deprecate multi_dof_segment.
  - Its functionality has been introduced in the QuinticSplineSegment rework that
  made it inherently multi-dof.
* Rename file for more consistent naming.
* Trivial doc fix.
* Trivial doc addition.
* Move controller-specific code to its namespace.
  ...and out of the trajectory_interface namespace.
* Better printing of time values.
* Move goal handle status setting to controller.
  - Basic state constraint validation remains in segment header file.
  - Goal handle status management now lives in controller.
  - Document constraint validation classes/functions.
* Fix current action goal resetting.
* Complete action interface support.
  - Goal completion is checked.
  - Tolerance checking implemented, needs cleanup, doc and testing though.
  - Reading tolerance data from goal handle and ROS param server is still TODO.
* QuinticSplineSegment is now inherently multi-dof.
  - MultiDofSegment wrapper is no longer needed.
  - Position, velocity, acceleration aata is now stored in a SoA structure, which is
  easier to manipulate.
* Topic interface support, partial action support.
  - Listening to command topics implemented.
  - Listening to action goals is implemented, but goal completion (success or
  aborted) is not implemented yet. Requires joint tolerances to be in place.
  - Add missing bits to make the plugin load.
* Use resize and [] instead of push_back on init.
* Add documentation.
* Unit test case where wraparound spec is ignored.
* Use uniform naming for wraparound variables.
* change initJointTrajectory() signature.
  - Instead of having 5 parameters: 2 madatory and 3 optional, now there are
  3 parameters: the same 2 mandatory, and an Options class with optional
  data. Specifying the options is optional.
  - Update unit tests.
* Complete feature set of initJointTrajectory()
  - Implement support for wrapping joints.
  - Some function parameters are now optional.
  - Comprehensive debug log statements.
  - Update unit test suite.
* Refactor initJointTrajectory()
  - Rename from init().
  - Move to a separate header, combine with code in controller.
  - Complete wrapping joint support, unit test it.
  - The unit test of initJointTrajectory() is commented-out. Needs to be updated.
* Add size() method.
* Reimplement permutation() leveraging STL more.
* Trivial log statement fixes.
* Add JointTrajectory segment construction options.
  - New optional parameter: Permutation vector, useful when joints in ROS message
  are not ordered as the controller expects them.
  - New optional parameter: Position offset, useful for handling joints that wrap
  around (ie. continuous), to compensate for multi-turn offsets.
  - Propagate changes to user classes and tests. Position offsets are not fully
  integrated yet.
* Add joint reordering on ROS message traj init.
  - When creating joint trajectories from ROS messages, it is now (optionally)
  possible to provide a vector of expected joint names, so expected joint count and
  names can be checked. Also, resulting trajectories are ordered as the expected
  joints vector, and not like the ROS message.
  - Update unit tests.
* Add tests for non-ros segment constructor.
* Implement init() method, except ROS API setup.
* Draft implementation of splicer, controller update.
  - Splicer code might be factored out soon, this is just a test.
* Add missing bits to conform with Segment API.
* Trivial doc fix.
* Better debug log statements.
* findSegment overload returning non-const iterator.
* Documentation and log message fixes.
* Implement trajectory init from ROS message data.
  Update unit tests accordingly.
* Add new trajectory segment type.
  - Multi-dof quintic spline segment that can be constructed from ROS message
  data structures.
* Scope init method as protected.
* Restructure error reporting log statements.
* Move trajectory_interface to a separate directory.
  General-purpose trajectory_interface functionality should live in a separate
  place (ie. ROS package) than the joint_trajectory_controller. This is a first
  step towards this separation.
* Add utilities for reading trajectories from ROS msgs.
* Add a non-iterator based findSegment(...) overload.
  Used for convenience purposes.
* Harmonize MultiDofSegment constructor signature.
  Segment implementations should have a similar way to be constructed, namely
  by specifying four parameters:
  start_time, start_state, end_time, end_state
  The types of these parameters depends on the segment type, but the structure
  is preserved.
* Trivial doc fix.
* Test trajectories with overlapping segments.
  Excercise the case when a segment's end_time is greater than the next segment's
  start_time.
* Remove dead code.
* Doc fixes.
* First trajectory interface functions with tests.
  - Find the segment associated to a specified time instant.
  - Sample a trajectory at a specified time instant.
* Add unit tests for segment sampling classes.
* Add doc configuration file.
* Initialize segments with start and end times.
  Before the segment duration was provided and zero start time was assumed.
  Now initializing and sampling segments have a more consistent API:
  - You initialize a segment from start and end time-state pairs.
  - You sample a segment with a time[in] and a state[out].
* Add basic trajectory interface and utilities.
  - Implementation of single-dof quintic spline segment.
  - Multi-dof segment templated on the single-dof segment type.
  - General(ish) representation of trajectories, templated on segment type.
* Layout skeleton if JointTrajectoryController.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.2 (2013-08-06)
------------------

0.5.1 (2013-07-19)
------------------

0.5.0 (2013-07-16)
------------------

0.4.0 (2013-06-26)
------------------
