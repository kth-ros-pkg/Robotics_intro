^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hardware_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.6 (2018-01-08)
------------------
* deleted changelogs
* maded joint mode not claim resources
* added joint mode empty constructor
* fixed missing return types and broken merge
* 0.2.5
* Updated changelog
* added pointer getters for torque sensor and absoute position encoders
* Added ptr accesors
* 0.2.4
* Updated changelog
* Added joint mode interface
* 0.2.3
* Updated changelog
* Removed print statement and const from hast torque sensor
* Added backcompatibility
* Absolute position and torque sensor working
* Modified structures to have absolute encoder and torque sensor parameters
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
* Fix argument types to use enum
* hardware_interface: fix initialization order
* Created new hardware interface for switching between controller modes
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
* Merge pull request #235 from shadow-robot/combined_robot_hw_kinetic
  Combined robot hw kinetic
* Add packages combined_robot_hw and combined_robot_hw_tests. combined_robot_hw allows to load different RobotHW as plugins and combines them into a single RobotHW. A single controller manager can then access resources from any robot.
* Allow the InterfaceManager class to register other InterfaceManagers.
  This will make it possible to combine several RobotHW objects into a single one.
* 0.10.1
* Update changelogs
* Fix rosconsole errors from test build
* 0.10.0
* Update changelogs
* Merge pull request #210 from ros-controls/fix-do-switch-jade
  controller_manager: Fix doSwitch execution point - jade
* controller_manager: Fix doSwitch execution point
  The doSwitch method needs to be executed in the update() method,  that is, in
  the real-time path, which is where controller switching actually takes place.
  It was previously done in the switchController callback, which is non real-time.
  In this method controller switching is scheduled, but not actually executed.
  This changeset fixes a bug in which hardware interface  modes could switch
  before controllers, leading to undefined behavior.
* Merge pull request #218 from ros-controls/prepare-switch-jade
  Prepare switch jade
* Deprecate RobotHW::canSwitch
  Has been superceeded by RobotHW::prepareSwitch.
* Cosmetic: Wrap method args across multiple lines
* Introduce prepareSwitch, replacement of canSwitch
  RobotHW::prepareSwitch is intended as a substitute for RobotHW::canSwitch.
  The main reasons for the change are a non-const signature to allow
  changing state and a more descriptive name.
  RobotHW::canSwitch will be deprecated in a later ROS distro.
* Merge pull request #204 from ros-controls/multi-iface-ctrl
  Allow controllers to claim resources from multiple interfaces
* hardware_interface: Add InterfaceManager::getNames
  Add new method that allows to query the names of all interfaces managed by
  an InterfaceManager instance.
* hardware_interface: Multi-interface controllers
  - This is a C++ API breaking changeset.
  - Modify ControllerInfo class to allow controllers to claim resources from
  multiple hardware interfaces.
  - Propagate changes to RobotHW::checkForConflict: Default resource ownsership
  policy is aware of controllers claiming resources from  multiple hardware
  interfaces.
  - Update and extend the corresponding test suite.
* Merge pull request #205 from ros-controls/w-unused-parameter
  Address -Wunused-parameter warnings
* Address -Wunused-parameter warnings
* Contributors: Adolfo Rodriguez Tsouroukdissian, Bence Magyar, Dave Coleman, Hilario Tome, Mathias Lüdtke, Paul Mathieu, Sam Pfeiffer, Toni Oliver, Victor Lopez

0.9.3 (2015-05-05)
------------------
* Update changelogs
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.2 (2015-05-04)
------------------
* Update changelogs
* Merge pull request #200 from ipa-mdl/strict_hwi_switch
  HW interface switch feature with unit tests
* added HW interface switch feature with unit tests
* Contributors: Adolfo Rodriguez Tsouroukdissian, Mathias Lüdtke

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
* Merge pull request #181 from ipa-mdl/pos_vel_acc
  Rebased  #148 (Added pos+vel and pos+vel+acc command interfaces)
* Add PosVel and PosVelAcc interfaces.
* Merge pull request #173 from shadowmanos/indigo-devel
  Fix spelling errors
* fix spelling errors
* Contributors: Adolfo Rodriguez Tsouroukdissian, Igorec, shadowmanos

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
* Merge pull request #156 from pal-robotics/transmission-loader-indigo
  Implement transmission loading from URDF - Indigo
* Merge pull request #155 from pal-robotics/indigo-devel
  Remove rosbuild artifacts. Fix #154.
* Fix doc typo.
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
* Merge pull request #147 from Igorec/hardware_interface
  Add ResourceHandle typedef
* Add ResourceHandle typedef
* Merge branch 'hydro-devel' of github.com:ros-controls/ros_control into hydro-devel
* Merge pull request #142 from pal-robotics/fix-cppcheck
  Add name to anonymous objects to avoid cppcheck error
* add name to anonymous objects to avoid cppcheck error
* Contributors: Adolfo Rodriguez Tsouroukdissian, Daniel Pinyol, Dave Coleman, Igorec

0.6.0 (2014-02-05)
------------------
* Updated changelogs
* Update interface_manager.h
  Trivial doc fix
* Merge pull request #135 from pal-robotics/actuator-interface-additions
  Add raw data accessors to actuators interface.
* Merge pull request #134 from pal-robotics/interface-manager
  Interface manager
* Add raw data accessors to actuators interface.
  Write access to the raw actuator data will be needed for automatic transmission
  loading.
* Fix doc typo.
* Migrate RobotHW class to use InterfaceManager.
* Factor out interface management parts of RobotHW.
  - Interface management is needed in the transmission_interface package as well.
  - Add new InterfaceManager internal class, with tests.
  - RobotHW remains untouched.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.8 (2013-10-11)
------------------
* "0.5.8"
* Updated changelogs
* Merge pull request #118 from ros-controls/no_manifest_xml
  Renamed manifest.xml to prevent conflicts with rosdep
* Merge branch 'hydro-devel' into extended_wait_time
* Merge pull request #121 from pal-robotics/hydro-devel
  Fixes for next minor release
* Renamed manifest.xml to prevent conflicts with rosdep
* Merge pull request #114 from vmayoral/hydro-devel
  CMakeLists fix to fit with OpenEmbedded/Yocto meta-ros layer.
* Move from postfix to prefix increment in loops.
  Detected by cppcheck 'postfixOperator' warning.
* CMakeLists fix to fit with OpenEmbedded/Yocto meta-ros layer.
  Increase the compatibility of the ros_control code with
  meta-ros, an OpenEmbedded/Yocto layer that provides recipes for ROS
  packages disabling catking checking the variable CATKIN_ENABLE_TESTING.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, vmayoral

0.5.7 (2013-07-30)
------------------
* Updated changelogs
* Contributors: Dave Coleman

0.5.6 (2013-07-29)
------------------
* Updated changelogs
* Merge branch 'hydro-devel' of github.com:ros-controls/ros_control into hydro-devel
* Updated changelogs
* Author/maintainer list update.
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
* Contributors: Dave Coleman

0.5.1 (2013-07-19)
------------------
* Typo fix
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
* Fix compiler warnings (-Wreorder)
* Merge pull request #77 from pal-robotics/hardware_interface_sensors
  Add interfaces for force-torque and IMU sensors
* Remove unused headers.
* Merge commit '69e64ff' into hardware_interface_sensors
* Merge commit '9bf48bd' into hardware_interface_sensors
* Merge commit '8054913' into hardware_interface_sensors
* Unit test sensor interfaces.
* Add default constructors to sensor handles.
* Tests build.
* Merge pull request #71 from davetcoleman/hydro-devel
  Renamed Github repos in docs, better error checking for spawning controllers
* Reneamed Github repo in documentation to ros-controls
* Merge branch 'fuerte_backport' into sensor_interfaces
* Merge branch 'master' of github.com:willowgarage/ros_control
* Merge branch 'master' into sensor_interfaces
* Add missing brace.
* Merge branch 'master' into sensor_interfaces
* Update sensor interfaces implementation.
  - Use resource managing classes introduced in recent hardware interface rework.
  - Conform to unified public API.
* Merge branch 'master' into sensor_interfaces
* Merge branch 'master' into sensor_interfaces
* Merge branch 'master' into sensor_interfaces
* Remove Eigen dependency from hardware_interface.
  - Expose force-torque and IMU sensor data as const pointers to the raw data.
  - Client code should wrap raw data however they prefer.
* Explicitly initialize IMU sensor handle members.
* Scrape orientation interface prototype.
* Merge branch 'master' into sensor_interfaces
* Add sensor ref frame field and capability queries.
* Add sensor reference frame field.
* First draft of sensor interfaces.
  - Force/torque (wrench)
  - Orientation
  - IMU (very crude approximation)
* Contributors: Adolfo Rodriguez Tsouroukdissian, Austin Hendrix, Dave Coleman

0.4.0 (2013-06-25)
------------------
* Version 0.4.0
* 1.0.1
* Merge pull request #66 from pal-robotics/master
  Fix duplicate header guard + minor maintenance.
* Add another convenience symbol demangling method.
  We already had:
  string foo_name = demangledTypeName<FooType>();
  which works great for typenames, but we were missing the equivalent for specific
  instances:
  FooType foo;
  string foo_name = demangledTypeName(foo);
  ...which works well for polymorphic types, returning the derived-most name.
* Fix duplicate header guard.
* Merge branch 'master' of github.com:willowgarage/ros_control
* Merge pull request #63 from pal-robotics/master
  Fix package URLs in package.xml
* Fix package URL in package.xml
* Merge pull request #62 from pal-robotics/master
  Update Doxygen doc, fix compiler warning.
* Fix compiler warning (-Wreorder).
* Merge pull request #59 from pal-robotics/master
  Documentation and log message improvements
* Restore documentation of handle parameters.
  Documentation that was previously in the interface classes before the
  hardware interface rework has been moved to the handle classes.
* Merge branch 'hardware_interface_rework'
* Fix ResourceManager exception messages.
  - Print derived class name instead of the less descriptive and more cryptic
  base class name. Eg.
  "hardware_interface::JointCommandInterface"
  instead of
  "hardware_interface::ResourceManager<hardware_interface::JointStateHandle>"
* Trivial doc/whitespace fix.
* Merge pull request #54 from pal-robotics/hardware_interface_rework
  Hardware interface rework
* Merge branch 'master' into hardware_interface_rework
  Conflicts:
  hardware_interface/CMakeLists.txt
* Separate resource manager in two classes.
  - Refs #45.
  - HardwareInterface specifics (ie. resource claiming) has been factored out.
  We now have the non-polymorphic ResourceManager class for registering and
  getting handles, and the polymorphic HardwareResourceManager that
  additionally implements the HardwareInterface and takes care of resource
  claiming.
  - The above change is required if the transmission interface is to leverage
  the resource management code, but without the hardware interface specifics.
  - Move files back to the internal folder. They are building blocks of the
  public API of hardware interfaces, but should not be directly #included
  by end users, so it's best they don't share the same location as
  user-facing headers.
  - Update unit tests.
* Add missing include statement.
* Validate raw data wrapped by hardware interfaces.
  - Refs #47 and #52.
  - Initialize raw data pointers to 0 in default handle constructors, otherwise
  they evaluate to nonzero and there is no way to distinguish an uninitialized
  state (ie. dangling pointers) from a properly initialized one.
  - For non-empty handle constructors, validate input raw data, throw if invalid
  pointers are found.
  - Add assertions on handle accessors. Invalid reads will trigger the assertions
  instead of causing a segfault (in debug mode).
  - Update unit tests.
* Warn when replacing a handle/interface.
  It is legitimate to change the underlying data associated to a handle/interface
  name, but it might also be a common programming error. Having the logs reflect
  this situation would allow to spot it easily.
* Make error message more explicit in test.
  Output with ROS_ERROR_STREAM instead of std::cout
* Add RobotHW class test.
* Add virtual destructor, protected internals.
  - ResourceManager inherits from HardwareInterface, which has virtual methods,
  so a virtual destructor is required.
  - Internal members are protected instead of private.
* Unit test hardware_interfaces.
* More uniform hardware_interface API. Refs  #45.
* Merge pull request #51 from jhu-lcsr-forks/master
  Adding cmake install targets
* adding install targets
* Merge pull request #49 from pal-robotics/master
  Restore joint resource claiming!.
* Restore joint resource claiming!.
  It had been mistakenly removed in a previous commit.
* Merge pull request #40 from jhu-lcsr-forks/catkin
  catkinizing, could still be cleaned up
* merging CMakeLists.txt files from rosbuild and catkin
* adding hybrid-buildsystem makefiles
* Merging from master, re-adding manifest.xml files
* Merge pull request #46 from pal-robotics/master
  Fix package URLs in manifest
* Fix package URLs.
* Merge pull request #44 from pal-robotics/master
  Fix exception throwing logic.
* Fix exception throwing.
* Merge pull request #43 from pal-robotics/master
  Harmonize how variables are quoted in log statements. Fixes #42.
* Harmonize how variables are quoted in logs.
  - Unify to using 'single quotes'.
  - Fixes #42.
* Merge pull request #41 from pal-robotics/master
  Add explicit actuators interface, create internal folder/namespace for non-public API
* Merge branch 'master' of https://github.com/willowgarage/ros_control
  Conflicts:
  hardware_interface/include/hardware_interface/joint_command_interface.h
* Add explicit actuator hardware interfaces.
  - These classes are similar to the existing joint equivalents, and are useful
  in setups leveraging the transmission_interface.
* Refactor named resource management code.
  - In preparation for the explicitly typed actuators interface, code for managing
  named resources has been refactored into a separate class. This code consists
  of convenience methods wrapping a std::map container, and occur often enough
  that factoring it out to prevent duplication makes sense.
  - Code that is not part of the public API, and hence with no stability guarantees
  has been moved to the internal folder/namespace. It only affects the named
  resource management and symbol demanglind methods so far.
* catkinizing, could still be cleaned up
* add accessor for command
* Remove redundant semicolons.
* Merge pull request #37 from pal-robotics/master
  Issue #36 fix.
* Use demangled type names when available. Fixes #36.
  Type names are used in different interfaces  such as hardware_interface and
  controller_interface. When symbol demangling is available (currently gcc 3.0+),
  operate on demangled names, as they are more convenient for human reading, eg.
  hardware_interface::VelocityJointInterface
  instead of
  N18hardware_interface22VelocityJointInterfaceE
* Merge pull request #30 from pal-robotics/master
  Documentation improvements
* Fix typo in rosdoc config files.
* Merge branch 'master' of github.com:willowgarage/ros_control into transmission_interface
* Merge pull request #27 from jbohren-forks/fix-joint-cmd-interface-error-msg
  Fixing error message in JointCommandInterface
* Fixing error message in JointCommandInterface
* Merge branch 'master' of github.com:willowgarage/ros_control into transmission_interface
* Merge pull request #23 from jbohren-forks/inline-doc
  Adding lots of inline documentation, rosdoc files
* More documentation in hardware_interface
* Adding template parameter doc
* Changing @ commands to \ commands
* Adding lots of inline documentation, rosdoc files
  adding inline doc to robot_hw
  adding inline doc to robot_hw
  adding inline doc to robot_hw
  more doc
  more documentation
  more doc
  more doc
  more doc
  more doc
  formatting
  adding more doc groups in controller manager
  adding more doc groups in controller manager
  Adding doc for controllerspec
  adding hardware interface docs
  adding doc to joint interfaces
  adding rosdoc for controller_interface
  Adding / reformatting doc for controller interface
* Merge pull request #20 from pal-robotics/master
  Minor missing header and ROS package dependencies
* Add missing explicit header dependency.
  Don't get required header transitively, but explicitly.
* Add mising roscpp dependency.
* cleanup
* move realtime tools in ros control, and create empty constructors for handles
* Doing resource conflict check on switchControllers call
* Adding in resource/claim infrastructure
* Refactoring joint command interfaces. Also added getJointNames()
* Switching to owned interfaces, instead of multiple virtual inheritance
* Changing interface names
* joint interfaces now throw on null joint value ptrs
* JointState is now JointMeasurement, to prevent naming collisions with pr2_mechanism
* Fixing copyright header text
* Joint interfaces now operate on pointers, instead of refs
* Merge branch 'fuerte'
* Tweaking inheritance to be virtual so it compiles. dummy app with controller manager compiles
* started controller_manager_tests. untested
* all pkgs now ported to fuerte
* hardware interface ported to fuerte
* more renaming
* new naming scheme
* running controller with casting. Pluginlib still messed up
* add macro
* running version, with latest pluginlib
* compiling version
* untested stuff, debians are screwed up
* compiling version
* first catkin stuff
* Contributors: Adolfo Rodriguez Tsouroukdissian, Austin Hendrix, Dave Coleman, Jonathan Bohren, Realtime Dev, Vijay Pradeep, Wim Meeussen, wmeeusse
