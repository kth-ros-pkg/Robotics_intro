^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package transmission_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.6 (2018-01-08)
------------------
* deleted changelogs
* fixed tests
* fixed missing return types and broken merge
* 0.2.5
* Updated changelog
* added pointer getters for torque sensor and absoute position encoders
* 0.2.4
* Updated changelog
* Fixed bug hasTorqueSensor and hasAbsoluteSensor, added std run_time exception support to controller manager
* 0.2.3
* Updated changelog
* Added has torque sensor and absolute encoder support to transmissions
* Added backcompatibility
* Absolute position and torque sensor working
* Modified structures to have absolute encoder and torque sensor parameters
* 0.2.2
* Update changelog
* Changed private members of transmission parser from private to protected in order to implement transmission parser with blacklist
* 0.2.1
* Update changelog
* 0.2.0
* Update changelog
* Allow loading from TransmissionInfo list
* 0.1.2
* Update changelogs
* 0.1.1
* Update changelogs
* 0.1.0
* Update changelogs
* Reset changelogs and version for pal release cycle
* Catkin fixes.
* Fix bug when adding multiple transmissions.
  - std::vectors were being used to store raw joint data, and when new transmissions
  were added, push_back()s would (potentially) reallocate the vectors and
  invalidate already stored pointers in hardware_interfaces. We now use std::map.
  - Move plugin implementations to a separate library.
  - Export link libraries to the outside.
  - More complete tests.
* First draft of transmission loading.
  - Only simple transmission type currently supported.
  - Can load forward map for act->jnt state and jnt->act pos,vel.eff commands.
  - Partial testing.
* Allow multiple hw interfaces, Fix #112, and test.
  - Allow to specify multiple hardware interfaces for joints and actuators.
  - Fix invalid xml_element tag. Contents are now stored as a string.
  - Unit test parser.
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
* 0.10.1
* Update changelogs
* Remove control_toolbox dependency. Fix thread linking error coming from removal of dependency.
* 0.10.0
* Update changelogs
* Merge pull request #204 from ros-controls/multi-iface-ctrl
  Allow controllers to claim resources from multiple interfaces
* Merge pull request #206 from ros-controls/load-trans
  transmission_interface: Add new loading method
* Merge pull request #205 from ros-controls/w-unused-parameter
  Address -Wunused-parameter warnings
* transmission_interface: Add new loading method
  Allow loading transmissions from a vector of TransmissionInfo instances.
* Address -Wunused-parameter warnings
* Contributors: Adolfo Rodriguez Tsouroukdissian, Bence Magyar, Hilario Tome, Hilario Tomé, Sam Pfeiffer, Victor Lopez

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
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.0 (2014-10-31)
------------------
* Update changelogs
* Merge pull request #186 from pal-robotics/catkin-lint-fixes
  Buildsystem fixes suggested by catkin_lint
* Buildsystem fixes suggested by catkin_lint
* Merge pull request #178 from pal-robotics/tr-iface-cmake
  Misc transmission_interface build fixes.
* Fix PLUGINLIB_DECLARE_CLASS depreacation warnings.
* Export missing libraries.
* Merge pull request #173 from shadowmanos/indigo-devel
  Fix spelling errors
* fix spelling errors
* Contributors: Adolfo Rodriguez Tsouroukdissian, shadowmanos

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
* Add developer documentation.
* Build script fixes.
  - Add missing libraries to catkin_package call.
  - Gate tests with CATKIN_ENABLE_TESTING.
  - Add missing files to install target.
* Fix possible memory corruption in tests.
* Perform sanity checks on members, not parameters.
  - The result is the same, but this is more uniform with the rest of the code.
* Enable joint reduction spec for 4-bar linkages.
  - As in the differential transmission, it's convenient to specify an additional
  mechanical reduction on the joint output. This is especially convenient for
  flipping the rotation direction of a joint (negative reduction value).
  - Update URDF loader.
  - Update documentation and tests.
* Trivial, cosmetic fixes.
* C++11 compatibility fixes.
* Fix resource check for multi-dof transmisisons.
* Efficiency fix.
  - cppcheck flagged a [passedByValue] warning. Using const references instead.
* Fix compiler warning.
* Fix license header in some files.
* Test transmission handle duplication.
* Use less pointers in transmission loader data.
  - Only RobotHW and RobotTransmission instances are pointers as they are owned
  by the robot hardware abstraction. The rest are plain members whose lifetime
  is bound to the loader struct.
* Trivial test addition.
* Remove unnecessary header dependencies.
* Catkin fixes.
* Fix bug when adding multiple transmissions.
  - std::vectors were being used to store raw joint data, and when new transmissions
  were added, push_back()s would (potentially) reallocate the vectors and
  invalidate already stored pointers in hardware_interfaces. We now use std::map.
  - Move plugin implementations to a separate library.
  - Export link libraries to the outside.
  - More complete tests.
* Log message change.
* Test greceful error-out with unsupported features.
* Add four-bar-linkage transmission parser.
* Add differential drive transmission parser.
* Move common XML parsing code to TransmissionLoader
  Mechanical reductions, offsets and roles are used by many transmission types.
  The TransmissionLoader base class exposes convenience methods for parsing these
  elements.
* Remove dead code.
* Update loader test, better log statements.
* First draft of transmission loading.
  - Only simple transmission type currently supported.
  - Can load forward map for act->jnt state and jnt->act pos,vel.eff commands.
  - Partial testing.
* Add class for holding transmission interfaces.
  - Mirrors hardware_interface::RobotHW, but for transmissions.
* Allow multiple hw interfaces, Fix #112, and test.
  - Allow to specify multiple hardware interfaces for joints and actuators.
  - Fix invalid xml_element tag. Contents are now stored as a string.
  - Unit test parser.
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
* Merge pull request #136 from pal-robotics/transmission-accessor-additions
  Add accessors to get transmission configuration.
* Add accessors to get transmission configuration.
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
* CMakeLists fix to fit with OpenEmbedded/Yocto meta-ros layer.
  Increase the compatibility of the ros_control code with
  meta-ros, an OpenEmbedded/Yocto layer that provides recipes for ROS
  packages disabling catking checking the variable CATKIN_ENABLE_TESTING.
* Fix license header in some files.
* Fix cppcheck uninit'd variable warnings in tests.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, vmayoral

0.5.7 (2013-07-30)
------------------
* Updated changelogs
* Fix for building ros_control
* Updated CHANGELOG
* Contributors: Dave Coleman

0.5.6 (2013-07-29)
------------------
* Updated changelogs
* Merge pull request #105 from ros-controls/cmake_modules_dependency
  Removed the local FindTINYXML.cmake and switched to catkin's cmake_modules version
* Added TinyXML to catkin_package DEPENDS
* Removed the local FindTINYXML.cmake and switched to catkin's cmake_modules version
* Merge branch 'hydro-devel' of github.com:ros-controls/ros_control into hydro-devel
* Updated changelogs
* Merge pull request #100 from piyushk/patch-1
  Installed missing transmission_interface_library
* Installed missing transmission_interface_library
* Contributors: Dave Coleman, Piyush Khandelwal

0.5.5 (2013-07-23 17:04)
------------------------
* Updated changelogs
* Merge branch 'hydro-devel' of github.com:ros-controls/ros_control into hydro-devel
* Merge pull request #95 from ros-controls/fix_tinyxml
  ros_control not building on the build farm
* transmission_interface: fixup finding tinyxml
* Contributors: Dave Coleman, William Woodall

0.5.4 (2013-07-23 14:37)
------------------------
* Merge branch 'hydro-devel' of github.com:ros-controls/ros_control into hydro-devel
* Updated changelogs
* Merge pull request #97 from ros-controls/hydro-tinyxml-cmake
  Duplicated urdfdom's method of including tinyxml
* Changed captilization of vars to match cmake standards
* Duplicated urdfdom's method of including tinyxml
* Contributors: Dave Coleman

0.5.3 (2013-07-22 18:06)
------------------------
* Updated changelog
* Duplicated URDF's method of including tinyxml
* Contributors: Dave Coleman

0.5.2 (2013-07-22 15:00)
------------------------
* Updated CHANGELOGS
* Created changelogs for all packages
* Trivial cleanup
* Merge branch 'hydro-devel' of github.com:ros-controls/ros_control
* Merge pull request #94 from davetcoleman/hydro-devel
  Fix transmission interface tinyxml build error
* Merge branch 'hydro-devel' of github.com:ros-controls/ros_control
* tinyxml include dir fix
* Contributors: Dave Coleman

0.5.1 (2013-07-19)
------------------
* Added new maintainer
* Merge pull request #92 from davetcoleman/master
  Attempt to fix transmission interface tinyxml build error
* Attempt to fix transmission interface tinyxml build error
* Merge branch 'hydro-devel'
* Contributors: Dave Coleman

0.5.0 (2013-07-16)
------------------
* Merge branch 'hydro-devel' of github.com:ros-controls/ros_control into hydro-devel
* Merge pull request #88 from ros-controls/master
  Merge master into hydro-devel for release to bloom
* Minor Doxygen fixes.
  - Revert back to using \file instead of \brief, as the latter was documenting
  the namespace and not the file scope.
  - Escape angular brackets on XML tag documentation, as Doxygen was parsing them
  printing warnings.
  @davetcoleman
* Code consistency fixes.
  - Add missing header guard.
  - Make existing header guards comply with the NAMESPACE_CLASS_H convention.
  - Make Doxygen structural commands start with '\' instead of '@', as most of the
  new ros_control code.
  - Remove trailing whitespaces.
  - Remove commented-out code used for debugging.
* Build script fixes.
  - Add missing tinyxml dependency.
  - Drop unnecessary Boost dependency.
  - Add URDF parsing code to rosbuild.
* Merge branch 'master' of https://github.com/willowgarage/ros_control
* Merge pull request #84 from ros-controls/transmission_parsing
  Added transmission parsing of XML/URDF files
* Merge branch 'master' of github.com:ros-controls/ros_control into transmission_parsing
* Add meta tags to packages not specifying them.
  - Website, bugtracker, repository.
* Merge branch 'master' of https://github.com/willowgarage/ros_control
* Documentation improvements.
  - More consistency between transmission and joint limits interfaces doc.
  - Make explicit that these interfaces are not meant to be used by controllers,
  but by the robot abstraction.
* Merge pull request #81 from davetcoleman/master
  Pulled in changes in hydro-devel to master
* Transmission parsing
* Merged hydro-devel into master
* Fix doc typo. Refs #78.
* Tests build.
* Merge pull request #71 from davetcoleman/hydro-devel
  Renamed Github repos in docs, better error checking for spawning controllers
* Reneamed Github repo in documentation to ros-controls
* Merge pull request #70 from pal-robotics/master
  Make specific transmission interfaces proper types.
* Merge branch 'fuerte_backport' into sensor_interfaces
* Make specific transmission interfaces proper types.
  - Proper types instead of namespaces allow to provide less cryptic feedback.
  * Using typedefs:
  "transmission_interface::TransmissionInterface<transmission_interface::ActuatorToJointPositionHandle>"
  * Using a new type:
  "transmission_interface::ActuatorToJointPositionInterface"
  - Added error message printing to tests for manual inspection.
* Merge branch 'master' into sensor_interfaces
* Merge branch 'master' into sensor_interfaces
* Contributors: Adolfo Rodriguez Tsouroukdissian, Austin Hendrix, Dave Coleman

0.4.0 (2013-06-25)
------------------
* Version 0.4.0
* 1.0.1
* Merge branch 'master' of github.com:willowgarage/ros_control
* Merge pull request #62 from pal-robotics/master
  Update Doxygen doc, fix compiler warning.
* Update Doxygen examples with recent API changes.
* Merge pull request #61 from adolfo-rt/patch-1
  Update README.md
* Merge pull request #59 from pal-robotics/master
  Documentation and log message improvements
* Update README.md
  Move examples out of readme and into ros_control's wiki.
* Merge branch 'hardware_interface_rework'
* Trivial doc/whitespace fix.
* Merge pull request #54 from pal-robotics/hardware_interface_rework
  Hardware interface rework
* Merge branch 'master' into hardware_interface_rework
  Conflicts:
  hardware_interface/CMakeLists.txt
* Leverage ResourceManager in TransmissionInterface.
  - Refs #45 and #48.
  - Leverage hardware_interface::internal::ResourceManager to implement
  TransmissionInterface more compactly and consistently.
  - Update unit tests.
* Merge pull request #51 from jhu-lcsr-forks/master
  Adding cmake install targets
* adding install targets
* Merge pull request #40 from jhu-lcsr-forks/catkin
  catkinizing, could still be cleaned up
* adding missing manifests
* removing comment
* merging CMakeLists.txt files from rosbuild and catkin
* adding hybrid-buildsystem makefiles
* Merging from master, re-adding manifest.xml files
* Merge pull request #43 from pal-robotics/master
  Harmonize how variables are quoted in log statements. Fixes #42.
* Harmonize how variables are quoted in logs.
  - Unify to using 'single quotes'.
  - Fixes #42.
* catkinizing, could still be cleaned up
* Merge pull request #30 from pal-robotics/master
  Documentation improvements
* Merge pull request #29 from pal-robotics/master
  Rename TransmissionException class
* Group transmission types in a Doxygen module.
* Rename TransmissionException class.
  Rename TransmissionException to TransmissionInterfaceException. It is more
  verbose, but more consistent with the existing HardwareInterfaceException.
* Merge pull request #28 from pal-robotics/master
  Add transmission interface
* Merge branch 'transmission_interface' of https://github.com/pal-robotics/ros_control into transmission_interface
* Add additional minimal example to mainpage doc.
  Existing example was complete, but quite long. It's better to start with a
  small and simple example.
* Update README.md
  Add additional minimal example.
* Update package wiki URL.
* Update README.md
* Update README.md
* Trivial doc fix.
* Add main page to documentation.
  It includes an overview of the transmission_interface package, pointers to the
  more relevant classes, and a commented example.
* Make transmission interface more general.
  The previous API assumed that to map a variable like position, one only
  needed actuator and joint space position variables. Although this is often the
  case (eg. fully actuated/determined transmissions), this does not hold in
  general. Underactuated transmissions are a typical example of this.
  Now each map accepts full <position,velocity,effort> triplets for actuator and
  joint space variables, and uses only the ones it needs.
  Although the current API has gained in generality, it has lost some of the
  explicitness it had before. For instance, if only position variables are
  needed for a map, one still needs to pass the full triplet (velocity and
  effort variables can be empty).
  Finally, unit tests and documentation have been updated to reflect the changes.
* Merge branch 'transmission_interface' of https://github.com/pal-robotics/ros_control into transmission_interface
* Minor documentation building fixes.
  - Remove test folder from docs.
  - Add proper export element in manifest.
* Update transmission_interface/README.md
* Update transmission_interface/README.md
* Add readme file.
* Remove pure virtual method.
* Use \name commands in documentation.
* Add pthread dependency to tests.
  After moving from Ubuntu 10.04 to 12.04 these dependencies need to be explicitly
  stated in my dev machine. This should be looked upon in greater detail, as such
  dependecies should be taken care of by rosbuild.
* Remove dependency from manifest.
* Add transmission interface class and test.
* Add transmission accessors test.
* Remove unnecessary virtual keywords.
* Add credit statement in docs.
* Add comprehensive doc to implemented transmissions.
  - More desriptive overview.
  - Images depicting each transmission type. Binary pngs  are under version control
  instead of getting auto-generated in the Makefile as not all build environments
  may have the necessary svg->png filters.
  - Expressions governing transmissions in tabular form.
* Basic documentation for implemented transmissions.
* Document abstract Transmission class.
* Add basic support for mechanical transmissions.
  - Base transmission class with abstract interface.
  - Specializations for three common transmission types: simple, differential and
  four-bar-linkage.
  - Unit tests with exercising preconditions, black-box and white-box tests.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Austin Hendrix, Dave Coleman, Jonathan Bohren, wmeeusse
