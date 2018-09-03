^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package controller_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.6 (2018-01-08)
------------------
* deleted changelogs
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
* 0.11.4
* Update logs
* 0.11.3
* Update changelogs
* 0.11.2
* Update changelogs
* Merge pull request #253 from bmagyar/cmake_warnings_boost
  Remove boost from depends declaration to fix cmake warning
* Remove boost from depends declaration to fix cmake warning
* Merge pull request #254 from bmagyar/update_package_xmls
  Update package xmls
* sort dependencies
* Add Enrique and Bence to maintainer list
* Clean up export leftovers from rosbuild
* Convert to format2, fix dependency in cmake
* 0.11.1
* Update changelog
* Merge pull request #247 from miguelprada/fix_multi_interface_example
  Fix the example in the comments in multi_interface_controller.h.
* Fix the example in the comments in multi_interface_controller.h.
* 0.11.0
* Update changelogs
* 0.10.1
* Update changelogs
* 0.10.0
* Update changelogs
* Merge pull request #204 from ros-controls/multi-iface-ctrl
  Allow controllers to claim resources from multiple interfaces
* controller_interface: Add MultiInterfaceController
  MultiInterfaceController: Subclass of ControllerBase which allows to claim
  resources from up to four different hardware interfaces.
  It's similar in spirit to the single-interface Controller class, in that
  hardware interfaces are specified as template parameters. One difference is that
  the custom controller initialization init() methods are passed a RobotHW*
  parameter containing only the hardware interfaces requested by the controller
  (as opposed to the actual hardware interface for Controller, or a RobotHW*
  representing the _entire\_ robot for ControllerBase).
  Requested hardware interfaces are required by default, but can be made optional
  through a constructor flag.
* controller_interface: Remove pure virtual method
  - This is a C++ API breaking changeset.
  - Remove getHardwareInterfaceType() pure virtual method from ControllerBase
  class. This public method assumes a single hardware interface, and is only
  used by the Controller subclass.
  - Controller class still preserves the method, albeit protected and non-virtual.
* controller_interface: Multi-interface controllers
  - This is a C++ API breaking changeset.
  - Modify controller interfaces to allow for controllers that claim resources
  from more than one hardware interface.
* Merge pull request #205 from ros-controls/w-unused-parameter
  Address -Wunused-parameter warnings
* Address -Wunused-parameter warnings
* Contributors: Adolfo Rodriguez Tsouroukdissian, Bence Magyar, Hilario Tome, Miguel Prada, Sam Pfeiffer, Victor Lopez

0.9.3 (2015-05-05)
------------------
* Update changelogs
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.2 (2015-05-04)
------------------
* Update changelogs
* Contributors: Adolfo Rodriguez Tsouroukdissian

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
* Contributors: Adolfo Rodriguez Tsouroukdissian

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
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.7.0 (2014-03-28)
------------------
* Prepare 0.7
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.6.0 (2014-02-05)
------------------
* Updated changelogs
* Contributors: Dave Coleman

0.5.8 (2013-10-11)
------------------
* "0.5.8"
* Updated changelogs
* Merge pull request #118 from ros-controls/no_manifest_xml
  Renamed manifest.xml to prevent conflicts with rosdep
* Renamed manifest.xml to prevent conflicts with rosdep
* Add .gitignore file.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.7 (2013-07-30)
------------------
* Updated changelogs
* Contributors: Dave Coleman

0.5.6 (2013-07-29)
------------------
* Updated changelogs
* Merge branch 'hydro-devel' of github.com:ros-controls/ros_control into hydro-devel
* Updated changelogs
* Merge pull request #103 from pal-robotics/hydro-devel
  Documentation fixes.
* Documentation fixes.
  - Tag (non)realtime methods in ControllerBase.
  - Fix incorrect param name in Controller.
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
* Merge pull request #71 from davetcoleman/hydro-devel
  Renamed Github repos in docs, better error checking for spawning controllers
* Reneamed Github repo in documentation to ros-controls
* Merge branch 'fuerte_backport' into sensor_interfaces
* Merge branch 'master' into sensor_interfaces
* Merge branch 'master' into sensor_interfaces
* Merge branch 'master' into sensor_interfaces
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.4.0 (2013-06-25)
------------------
* Version 0.4.0
* 1.0.1
* Merge branch 'master' of github.com:willowgarage/ros_control
* Merge pull request #63 from pal-robotics/master
  Fix package URLs in package.xml
* Fix package URL in package.xml
* Merge branch 'master' into hardware_interface_rework
  Conflicts:
  hardware_interface/CMakeLists.txt
* Merge pull request #51 from jhu-lcsr-forks/master
  Adding cmake install targets
* adding install targets
* Merge pull request #40 from jhu-lcsr-forks/catkin
  catkinizing, could still be cleaned up
* merging CMakeLists.txt files from rosbuild and catkin
* adding hybrid-buildsystem makefiles
* Merging from master, re-adding manifest.xml files
* Merge pull request #46 from pal-robotics/master
  Fix package URLs in manifest
* Fix package URLs.
* Merge pull request #41 from pal-robotics/master
  Add explicit actuators interface, create internal folder/namespace for non-public API
* Refactor named resource management code.
  - In preparation for the explicitly typed actuators interface, code for managing
  named resources has been refactored into a separate class. This code consists
  of convenience methods wrapping a std::map container, and occur often enough
  that factoring it out to prevent duplication makes sense.
  - Code that is not part of the public API, and hence with no stability guarantees
  has been moved to the internal folder/namespace. It only affects the named
  resource management and symbol demanglind methods so far.
* catkinizing, could still be cleaned up
* Merge pull request #37 from pal-robotics/master
  Issue #36 fix.
* Use demangled type names when available. Fixes #36.
  Type names are used in different interfaces  such as hardware_interface and
  controller_interface. When symbol demangling is available (currently gcc 3.0+),
  operate on demangled names, as they are more convenient for human reading, eg.
  hardware_interface::VelocityJointInterface
  instead of
  N18hardware_interface22VelocityJointInterfaceE
* Merge pull request #34 from pal-robotics/master
  Minor documentation improvements
* [Trivial] Remove redundant semicolon.
* Update controller_interface docs.
  More descriptive documentation for initialization methods with two NodeHandle
  arguments.
* add option to pass in two nodehandles to a controller: one in the root of the controller manager namespace, and one in the namespace of the controller itself. This copies the behavior used by nodelets and nodes
* Merge pull request #30 from pal-robotics/master
  Documentation improvements
* Fix typo in rosdoc config files.
* Merge branch 'master' of github.com:willowgarage/ros_control into transmission_interface
* Merge pull request #23 from jbohren-forks/inline-doc
  Adding lots of inline documentation, rosdoc files
* Adding template parameter doc
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
* Merge branch 'master' of github.com:willowgarage/ros_control
* new interface with time and duration
* cleanup
* Adding in resource/claim infrastructure
* clean up publishing controller state
* Switching to owned interfaces, instead of multiple virtual inheritance
* Fixing copyright header text
* Merge branch 'fuerte'
* Tweaking inheritance to be virtual so it compiles. dummy app with controller manager compiles
* all pkgs now ported to fuerte
* running controller with casting. Pluginlib still messed up
* add macro
* compiling version
* move joint state controller to new package
* make a dummy plugin
* untested stuff, debians are screwed up
* compiling version
* working install target
* base classes
* first catkin stuff
* Contributors: Adolfo Rodriguez Tsouroukdissian, Austin Hendrix, Dave Coleman, Jonathan Bohren, Vijay Pradeep, Wim Meeussen, hiDOF, wmeeusse
