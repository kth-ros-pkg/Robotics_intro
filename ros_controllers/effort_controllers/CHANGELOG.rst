^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package effort_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge pull request #299 from Maik93/kinetic-devel
  added effort_controllers/JointGroupPositionController
* removed the already commented 'controller_state_publisher\_'
* implemented enforceJointLimits
* removed commented code
* compile succesfully in kinetic
* Remove unused-undefined
* Remove unused includes
* Allocate the necessary number of pid controllers
* Add JointGroupPositionController
* Contributors: Bence Magyar, Maik Mugnai

0.13.0 (2017-08-10)
-------------------
* Update changelogs
* Contributors: Bence Magyar

0.12.3 (2017-04-23)
-------------------
* Update changelogs
* Merge pull request #245 from piyushk/piyushk/provide_nh_to_urdf
  Supply nh to urdf::Model
* Supply NodeHandle to urdf::Model. Closes #244
* Contributors: Bence Magyar, Piyush Khandelwal

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
* Remove unused dependency
* Change for format2
* Add Enrique and Bence to maintainers
* Merge branch 'kinetic-devel' into F_enable_part_traj_kinetic
* Merge pull request #235 from bmagyar/unboost-urdf-fix
  Replace boost::shared_ptr<urdf::XY> with urdf::XYConstSharedPtr when exists
* Replace boost::shared_ptr<urdf::XY> with urdf::XYConstSharedPtr when exists
* Contributors: Bence Magyar, Enrique Fernández Perdomo, beatrizleon

0.11.2 (2016-08-16)
-------------------
* Update changelogs
* Included angles in dependencies
* Contributors: Bence Magyar, Mr-Yellow

0.11.1 (2016-05-23)
-------------------
* Update changelogs
* Contributors: Bence Magyar

0.11.0 (2016-05-03)
-------------------
* Update changelogs
* Merge pull request #214 from clearpathrobotics/antiwindup-fix
  Add antiwindup to get/setGains methods of jointPosition and jointVelocity controllers
* Add method with old signature to preserve ABI compatibility
* Add antinwindup to get and setGains logic for underlying PID controller
* Contributors: Bence Magyar, Paul Bovbel

0.10.0 (2015-11-20)
-------------------
* Update changelogs
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.2 (2015-05-04)
------------------
* Update changelogs.
* Merge pull request #161 from ipa-mdl/rt_group_controller
  thread-safe forward controllers
* thread-safe and realtime-safe  forward controllers
  This is a combination of 3 commits.
  * migrated to realtime_buffer
  * use RealtimeBuffer for ForwardCommandController
  * protected write at initialization
* Contributors: Adolfo Rodriguez Tsouroukdissian, Mathias Lüdtke

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
* Merge pull request #135 from ipa-fxm/test_initial_value_position_controller
  use current position when starting (forward_command) position_controllers
* use current position when starting (forward_command) position_controllers; effort and velocity still use 0.0
* Merge pull request #128 from ipa-fxm/forward_chain_command_controller
  add forward_chain_command_controllers
* add forward_chain_command_controllers
* Contributors: Adolfo Rodriguez Tsouroukdissian, ipa-fxm

0.8.1 (2014-07-11)
------------------
* Update chegelogs
* Merge pull request #96 from cottsay/indigo-devel
  Add depend on angles
* Add depend on angles
* Contributors: Adolfo Rodriguez Tsouroukdissian, Scott K Logan

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
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.6.0 (2014-02-05)
------------------
* Updated changelogs
* Merge branch 'hydro-devel' into joint_trajectory_tweaks
* Merge pull request #54 from davetcoleman/effort_position_controller_fix
  Added new has_velocity flag that indiciates if a target velocity has been set
* Merge branch 'hydro-devel' into development
* Added new has_velocity flag that indiciates if a target velocity has been set
* Merge branch 'hydro-devel' of https://github.com/willowgarage/ros_controllers into hydro-devel
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.4 (2013-09-30)
------------------
* Updated changelogs
* Contributors: Dave Coleman

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
* Merge pull request #35 from ros-controls/effort_controller_fixes
  Effort Controller Fixes
* Minor comment fix
* Critical bug: velocity controller init() does not get hardware_interface handle for joint
* Fixes for joint_position_controller
* Consolidated position and velocity command into one realtime buffer
* Tweaked header guard
* Added ability to set target velocity, CMake cleanup
* Merge pull request #33 from ros-controls/effort_position_controller_fix
  Effort position controller fix
* Removed debug output from realtime context
* Removed blocking msgs from realtime loop
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
* Merge pull request #30 from davetcoleman/master
  Removed controller_msgs, changed to control_msgs
* Merge pull request #29 from davetcoleman/hydro-devel
  Reviewed by @jbohren
  Fixed PID destructor bug, cleaned up code
* Removed controller_msgs
* Fixed PID destructor bug, cleaned up code
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.
* Restore "Fixed PLUGINLIB_DECLARE_CLASS depreacated errors""
  This reverts commit 0862ad93696b0d736b565cd65ea36690dde0eaa7.
* Merge pull request #26 from jhu-lcsr-forks/hydro-devel
  Fixing reversed error computation...
* Fixing reversed error computation...
* Merge pull request #25 from jhu-lcsr-forks/hydro-devel
  Adding install targets for plugin xml files
* Adding install targets for plugin xml files
* Merge branch 'fuerte_backport' into sensor_interfaces
* Revert "Fixed PLUGINLIB_DECLARE_CLASS depreacated errors"
  This reverts commit 2314b8b434e35dc9c1c298140118a004e00febd8.
* Merge branch 'hardware_interface_rework' into sensor_interfaces
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, Jonathan Bohren, wmeeusse

0.4.0 (2013-06-26)
------------------
* Version 0.4.0
* Merge pull request #22 from jhu-lcsr-forks/master
  Fixing position effort controller pid command args
* Fixing position effort controller pid command args
* Merge pull request #16 from davetcoleman/master
  Fixed control_toolbox deprecated errors with updatePid()
* Fixed control_toolbox deprecated errors with updatePid()
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
* Merge pull request #10 from pal-robotics/master
  Minor maintenance fixes.
* Trivial log message fix.
* Fixing library export
* adding these packages which weren't seen by catkinize_stack
* bumping version
* adding package.xml files
* Catkinizing. Building, but could still be cleaned up
* Merge pull request #7 from pal-robotics/master
  Extend joint_effort_controller to other interfaces
* Extend joint_effort_controller to other interfaces
  - Factor-out implementation of simple command-forwarding controller.
  - Provide specializations (typedefs really) for effort, velocity and position
  interfaces.
* Fix documentation typo.
* Merge pull request #6 from pal-robotics/master
  Minor maintenance fixes.
* Add .gitignore files on a per-package basis.
* effort_controllers::joint_velocity_controller was not being built
* Merge pull request #3 from jbohren-forks/master
  Fixing typos in JointVelocityController
* Fixing typos in JointVelocityController
* port to new api with time and duration
* fix xml filename
* register controllers
* fixes
* add position controller
* port another controller
* clean up dependencies
* first simple controller for testing
* Contributors: Adolfo Rodriguez Tsouroukdissian, Austin Hendrix, Dave Coleman, Jonathan Bohren, Wim Meeussen, hiDOF, wmeeusse
