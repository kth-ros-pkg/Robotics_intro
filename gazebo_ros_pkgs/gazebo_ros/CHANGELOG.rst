^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.7 (2018-01-12)
------------------

2.6.6 (2018-01-12)
------------------

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
* Prevents GAZEBO_MODEL_DATABASE_URI from being overwritten (#644)
* Fix gazebo8 warnings part 7: ifdef's for Joint::GetAngle and some cleanup (#642)
  * fix major version check >= 8, instead of > 8
  * gazebo_ros_bumper: use new API in commented code
  * gazebo_ros_api_plugin: world pose in local vars
  * worldLinearVel as local var in hand of god plugin
  * gazebo8+: Joint::GetAngle -> Joint::Position
* Contributors: Hilario Tome, R, Steven Peters

2.5.14 (2017-12-11)
-------------------
* Fix latin character not allowed failing by prepare_release script
* Generate changelogs
* for gazebo8+, call functions without Get (#639)
* Fix gazebo8 warnings part 5: ignition math in gazebo_ros (#635)
  * gazebo_ros: ign-math in private API, local vars
  * gazebo_ros: pass const reference instead of copy
* Fix gazebo8 warnings part 4: convert remaining local variables in plugins to ign-math (#633)
  * plugins: convert all local vars to ign-math
  * ft_sensor: fix gazebo7 build
  * Use World::[GS]etGravity
  * fix gravity syntax
* gazebo_ros: fix support for python3 (#622)
* gazebo_ros_api_plugin: improve plugin xml parsing (#625)
  An xml comment that start with plugin causes a seg-fault:
  <!--plugin-->
  or
  <!--plugin filename="lib.so"/-->
  This fixes the xml parsing to not try to add child elements
  to xml comments.
* Replace Events::Disconnect* with pointer reset (#623)
* Install spawn_model using catkin_install_python (#621)
* [gazebo_ros] don't overwrite parameter "use_sim_time" (#606)
  * Parameter /use_sim_time is only set if not present on Parameter Server
* Contributors: Jose Luis Rivero, Manuel Ilg, Mike Purvis, Nils Rokita, Steven Peters

2.5.13 (2017-06-24)
-------------------
* Update changelogs
* Quote arguments to echo in libcommon.sh (#590)
  If /bin/sh is provided by bash, echo will consume arguments such as `-e`. On such a system, running `rosrun gazebo_ros gzserver -e ode empty_world.world` will execute `gzserver` with the `-e` missing (meaning the world file is ignored).
* Add catkin package(s) to provide the default version of Gazebo - take II (kinetic-devel) (#571)
  * Added catkin package gazebo_dev which provides the cmake config of the installed Gazebo version
  Conflicts:
  gazebo_plugins/package.xml
  gazebo_ros/package.xml
  gazebo_ros_control/package.xml
  * gazebo_plugins/gazebo_ros: removed dependency SDF from CMakeLists.txt
  The sdformat library is an indirect dependency of Gazebo and does not need to be linked explicitly.
  * gazebo_dev: added execution dependency gazebo
* Contributors: Jose Luis Rivero, daewok

2.5.12 (2017-04-25)
-------------------
* Changelogs for next version
* Contributors: Jose Luis Rivero

2.5.11 (2017-04-18)
-------------------
* Changelogs to prepare for next 2.5.11
* [gazebo_ros] Changed the spawn model methods to spawn also lights. (#511)
  * [gazebo_ros] Changed the spawn model methods to spawn also lights (and renamed accordingly).
  Created services for deleting lights, and getting and settings lights' properties.
  * [gazebo_ros] Changed the spawn model methods to spawn also lights.
  Created services for deleting lights, and getting and settings lights' properties.
  * [gazebo_ros] Changed the spawn model methods to spawn also lights.
  Created services for deleting lights, and getting and settings lights' properties.
  * [gazebo_ros] Changed the spawn model methods to spawn also lights.
  Created services for deleting lights, and getting and settings lights' properties.
  * [gazebo_ros] Changed the spawn model methods to spawn also lights.
  Created services for deleting lights, and getting and settings lights' properties.
* Change build system to set DEPEND on Gazebo/SDFormat (fix catkin warning)
  Added missing DEPEND clauses to catkin_package to fix gazebo catkin warning. Note that after the change problems could appear related to -lpthreads errors. This is an known issue related to catkin: https://github.com/ros/catkin/issues/856.
* Use correct logerr method (#557)
* Contributors: Alessandro Ambrosano, Dave Coleman, Gary Servin, Jose Luis Rivero

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
* Merge pull request #539 from davetcoleman/kinetic-whitespace
  Removed all trailing whitespace
* Removed all trailing whitespace
* Contributors: Dave Coleman, Jose Luis Rivero

2.5.8 (2016-12-06)
------------------
* Update changelogs for 2.5.8
* Merge pull request #516 from ros-simulation/reorder_parameters
  Workaround to support gazebo and ROS arguments in the command line respecting ROS remappings.
* Use -q with grep and fix comments. Thanks to Martin Pecka.
* Workaround to support gazebo and ROS arguments in the command line
  Reorder command line arguments to place ROS remappings at the end so
  gazebo passed them to be handle by gazebo ROS plugins. While this
  is not the recommended way of using rosrun, it could be useful for
  some use cases.
* Merge pull request #514 from jonbinney/jb-revert-remapping-removal
  Make ROS remapping to work again by reverting the change "Remove ROS remapping arguments from gazebo_ros launch scripts."
* Revert "Remove ROS remapping arguments from gazebo_ros launch scripts."
  This reverts commit a90e609a81702b13bee235b079081edf68ff6971.
* Merge pull request #501 from ros-simulation/kinetic-devel-transplant-500
  gazebo_ros: replace 'headless' arg with 'recording' (kinetic-devel)
* Merge pull request #502 from ros-simulation/kinetic-devel-transplant-495
  Fixed getLinkState service's angular Z velocity return
* Fixed getlinkstate service's angular velocity return
* Added comments regarding 'headless' arg and issue #491. Added 'recording' arg as switch for -r
* Merge pull request #467 from ros-simulation/kinetic-devel-transplant-459
  launch scripts override GAZEBO_MASTER_URI (kinetic-devel)
* GAZEBO_MASTER_URI is loaded from setup.sh if empty in environment.
* Honor GAZEBO_MASTER_URI for gzserver.
* Honor GAZEBO_MASTER_URI for gzclient.
* launch scripts override GAZEBO_MASTER_URI
  GAZEBO_MASTER_URI is always the one written in `setup.sh`, even if a different value is set in the user's environment.
  I consider it counter-intuitive, if you e.g. run `GAZEBO_MASTER_URI=http://myserver:11345 rosrun gazebo_ros gzserver`, that the server is still started with the default URI.
* Contributors: Jared, Jon Binney, Jordan Liviero, Jose Luis Rivero, Martin Pecka

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
* Remove deprecated spawn_gazebo_model service
  It was deprecated in hydro.
  This fixes a compiler warning.
* Contributors: Jose Luis Rivero, Steven Peters

2.5.4 (2016-04-27)
------------------
* Update changelogs
* Merge pull request #454 from scpeters/merge_ijk
  merge indigo, jade to kinetic-devel
* merge indigo, jade to kinetic-devel
* Merge pull request #435 from ros-simulation/bond_caguero
  Replacement for #303
* Merge branch 'kinetic-devel' of https://github.com/ros-simulation/gazebo_ros_pkgs into kinetic-devel
* Upgrade to gazebo 7 and remove deprecated driver_base dependency (#426)
  * Upgrade to gazebo 7 and remove deprecated driver_base dependency
  * disable gazebo_ros_control until dependencies are met
  * Remove stray backslash
* Merge pull request #430 from ros-simulation/kinetic-devel-maintainer
  Update maintainer for Kinetic release
* spawn_model: adding -b option to bond to the model and delete it on sigint
* Update maintainer for Kinetic release
* Merge pull request #342 from 130s/impr/allow_respawn_gazebo
  [empty_world.launch] Allow respawning gazebo node.
* [empty_world.launch] Allow respawning gazebo node.
* Contributors: Hugo Boyer, Isaac IY Saito, Jackie Kay, Jonathan Bohren, Jose Luis Rivero, Steven Peters

2.5.3 (2016-04-11)
------------------
* Update changelogs for 2.5.3
* Merge pull request #390 from peci1/issue_387_remove_ros_remappings
  [gazebo_ros] Remove ROS remapping arguments from gazebo_ros launch scripts.
* Merge branch 'jade-devel' into issue_387_remove_ros_remappings
* Merge pull request #403 from ros-simulation/jade-devel-fix-testing-suite
  Include gazebo binary package as runtime dependency
* Include binary in runtime
* Remove ROS remapping arguments from gazebo_ros launch scripts.
* Contributors: Jose Luis Rivero, Martin Pecka

2.5.2 (2016-02-25)
------------------
* Prepare changelogs
* merging from indigo-devel
* Merge pull request #302 from maxbader/jade-devel-GetModelState
  Header for GetModelState service request for jade-devel
* Merge pull request #362 from ubi-agni/indigo-devel
  [gazebo_ros] fixes #361
* Fix invalid signal name on OS X
  scripts/gazebo: line 30: kill: SIGINT: invalid signal specification
* Merge pull request #364 from bgromov/fix_gazebo_sigint_osx
  [gazebo_ros] Fix invalid signal name on OS X for script/gazebo
* Fix invalid signal name on OS X
  scripts/gazebo: line 30: kill: SIGINT: invalid signal specification
* Restart package resolving from last position, do not start all over.
* 2.4.9
* Generate changelog
* Merge pull request #335 from pal-robotics-forks/add_range_sensor_plugin
  Adds range plugin for infrared and ultrasound sensors from PAL Robotics
* Merge pull request #350 from ros-simulation/indigo-devel_merged_from_jade
  Merge changes from jade-devel into indigo-devel
* Import changes from jade-branch
* Add range world and launch file
* Merge pull request #331 from iche033/fix_disconnect_event
  Fix crash due to world disconnect event
* fix crash
* Merge pull request #2 from ros-simulation/indigo-devel
  Indigo devel
* Merge pull request #314 from ros-simulation/gazebo_cpp11
  Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* GetModelState modification for jade
* Contributors: Bence Magyar, Boris Gromov, Guillaume Walck, Ian Chen, John Hsu, Jose Luis Rivero, Markus Bader, Nate Koenig, Steven Peters, hsu, iche033

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
* Merge pull request #331 from iche033/fix_disconnect_event
  Fix crash due to world disconnect event
* fix crash
* Merge pull request #2 from ros-simulation/indigo-devel
  Indigo devel
* Merge pull request #314 from ros-simulation/gazebo_cpp11
  Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* Contributors: Bence Magyar, Ian Chen, Jose Luis Rivero, Nate Koenig, Steven Peters, iche033

2.4.8 (2015-03-17)
------------------
* Generate new changelog
* Merge pull request #242 from ros-simulation/multi_physics
  Specify physics engine in args to empty_world.launch
* Specify physics engine in args to empty_world.launch
* Contributors: Jose Luis Rivero, Steven Peters

2.4.7 (2014-12-15)
------------------
* Changelogs for 2.4.7 branch
* Merge pull request #255 from ros-simulation/fix_gazebo_ros_tutorial_url
  Update Gazebo/ROS tutorial URL
* Merge pull request #238 from ayrton04/indigo-devel
  Fixing handling of non-world frame velocities in setModelState.
* Merge pull request #278 from k-okada/93_indigo
  temporary hack to **fix** the -J joint position option (issue #93), slee...
* temporary hack to **fix** the -J joint position option (issue #93), sleeping for 1 second to avoid race condition. this branch should only be used for debugging, merge only as a last resort.
* Fixing set model state method and test
* Merge pull request #247 from peci1/patch-1
  [gazebo_ros] Fix for #246
* Extended the fix for #246 also to debug, gazebo, gzclient and perf scripts.
* Update Gazebo/ROS tutorial URL
* [gazebo_ros] Fix for #246
  Fixing issue #246 in gzserver.
* Merge pull request #237 from ros-simulation/update_header_license
  Update header license for Indigo
* Fixing handling of non-world frame velocities in setModelState.
* update headers to apache 2.0 license
* update headers to apache 2.0 license
* Contributors: John Hsu, Jose Luis Rivero, Martin Pecka, Steven Peters, Tom Moore, ayrton04, hsu

2.4.6 (2014-09-01)
------------------
* Changelogs for version 2.4.6
* Merge pull request #227 from ros-simulation/fix_get_physics_properties_non_ode_hydro
  check physics engine type before calling set_physics_properties and get\_...
* Merge pull request #232 from ros-simulation/fix_get_physics_properties_non_ode
  Fix get physics properties non ode
* Merge pull request #183 from ros-simulation/issue_182
  Fix STL iterator errors, misc. cppcheck (#182)
* check physics engine type before calling set_physics_properties and get_physics_properteis
* check physics engine type before calling set_physics_properties and get_physics_properteis
* Fixes for calling GetParam() with different physic engines.
* 2.3.6
* Update changelogs for the upcoming release
* Merge pull request #221 from ros-simulation/fix_build
  Fix build for gazebo4
* Fixed boost any cast
* Removed a few warnings
* Update for hydro + gazebo 1.9
* Fix build with gazebo4 and indigo
* Fix STL iterator errors, misc. cppcheck (#182)
  There were some errors in STL iterators.
  Initialized values of member variables in constructor.
  Removed an unused variable (model_name).
* Merge remote-tracking branch 'origin/hydro-devel' into camera-info-manager
* Merge pull request #1 from ros-simulation/hydro-devel
  Merge from upstream
* Contributors: Carlos Agüero, John Hsu, Jonathan Bohren, Jose Luis Rivero, Nate Koenig, Steven Peters, hsu, osrf

2.4.5 (2014-08-18)
------------------
* Changelogs for upcoming release
* Merge pull request #222 from ros-simulation/fix_build_indigo
  Port fix_build branch for indigo-devel (fix compilation for gazebo4)
* Port fix_build branch for indigo-devel
  See pull request #221
* Contributors: Jose Luis Rivero, hsu

2.4.4 (2014-07-18)
------------------
* Update Changelog
* Merge branch 'hydro-devel' into indigo-devel
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Merge pull request #199 from Arn-O/hydro-devel
  change equality operator in rosrun scripts to be posix compliant
* Merge pull request #201 from jonbinney/indigo-repos
  Fix repository urls for indigo branch
* Merge pull request #202 from jonbinney/hydro-repos
  Fix repo names in package.xml's (hydro-devel branch)
* Fix repo names in package.xml's
* Fix repo names in package.xml's
* fix issue #198
  Operator ``==`` is not recognized by sh scripts.
* fix issue #198
  Operator ``==`` is not recognized by sh scripts.
* fix issue #198
  Operator ``==`` is not recognized by sh scripts.
* fix issue #198
  Operator ``==`` is not recognized by sh scripts.
* fix issue #198
  Operator ``==`` is not recognized by sh scripts.
* Merge remote-tracking branch 'origin/hydro-devel' into indigo-devel
* Merge pull request #190 from clynamen/patch-1
  Add verbose parameter
* Add verbose parameter
  Add verbose parameter for --verbose gazebo flag
* Merge pull request #188 from markusachtelik/hydro-devel
  added osx support for gazebo start scripts
* added osx support for gazebo start scripts
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Merge pull request #1 from ros-simulation/hydro-devel
  Merge from upstream
* Contributors: Arn-O, John Hsu, Jon Binney, Jonathan Bohren, Markus Achtelik, Markus Bader, Steven Peters, Vincenzo Comito

2.4.3 (2014-05-12)
------------------
* update changelog
* added osx support for gazebo start scripts
* update changelog
* Merge pull request #181 from ros-simulation/gazebo_plugins_undepend
  Reverse gazebo_ros dependency on gazebo_plugins
* Remove gazebo_ros dependency on gazebo_plugins
* Contributors: Markus Achtelik, Steven Peters

2.4.2 (2014-03-27)
------------------
* catkin_tag_changelog
* catkin_generate_changelog
* merging from hydro-devel
* 2.3.5
* catkin_tag_changelog
* catkin_generate_changelog and fix rst format for forthcoming logs
* Merge pull request #157 from pal-robotics/mini-fix
  Very small fix in gazebo_ros_api_plugin
* gazebo_ros: [less-than-minor] fix newlines
* gazebo_ros: remove assignment to self
  If this is needed for any twisted reason, it should be made clear
  anyway. Assuming this line is harmless and removing it because it
  generates cppcheck warnings.
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Contributors: Jim Rothrock, John Hsu, Paul Mathieu, hsu

2.4.1 (2013-11-13 18:52)
------------------------
* bump patch version for indigo-devel to 2.4.1
* merging from indigo-devel after 2.3.4 release
* "2.3.4"
* preparing for 2.3.4 release (catkin_generate_changelog, catkin_tag_changelog)
* Merge branch 'hydro-devel' of github.com:ros-simulation/gazebo_ros_pkgs into indigo-devel
* Merge pull request #150 from ros-simulation/spawn_model_pose_fix
  Spawn model pose fix
* remove debug statement
* fix sdf spawn with initial pose
* fix sdf spawn with initial pose
* Merge pull request #148 from ros-simulation/spawn_model_pose_fix
  fix spawn initial pose.  When model has a non-zero initial pose and user...
* Merge branch 'hydro-devel' into spawn_model_pose_fix
* Merge pull request #149 from ros-simulation/fix_indentation
  fix indentation
* fix indentation
* Merge pull request #142 from hsu/hydro-devel
  fix issue #38, gui segfault on model deletion
* Merge pull request #140 from v4hn/spawn_model_sleep
  replace time.sleep by rospy.Rate.sleep
* Merge pull request #137 from fsuarez6/patch-1
  Add time import
* Merge pull request #132 from po1/fix-iterators
  Fix iterator-related things
* fix spawn initial pose.  When model has a non-zero initial pose and user specified initial model spawn pose, add the two.
* fix issue #38, gui segfault on model deletion by removing an obsolete call to set selected object state to "normal".
* replace time.sleep by rospy.Rate.sleep
  time was not even imported, so I don't know
  why this could ever have worked...
* Add time import
  When using the -wait option the script fails because is missing the time import
* Use pre-increment for iterators
* Fix iterator erase() problems
* Contributors: Francisco, John Hsu, Paul Mathieu, hsu, v4hn

2.4.0 (2013-10-14)
------------------
* "2.4.0"
* catkin_generate_changelog
* Contributors: John Hsu

2.3.5 (2014-03-26)
------------------
* catkin_tag_changelog
* catkin_generate_changelog and fix rst format for forthcoming logs
* Merge pull request #157 from pal-robotics/mini-fix
  Very small fix in gazebo_ros_api_plugin
* gazebo_ros: [less-than-minor] fix newlines
* gazebo_ros: remove assignment to self
  If this is needed for any twisted reason, it should be made clear
  anyway. Assuming this line is harmless and removing it because it
  generates cppcheck warnings.
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Contributors: Jim Rothrock, John Hsu, Paul Mathieu, hsu

2.3.4 (2013-11-13 18:05)
------------------------
* "2.3.4"
* preparing for 2.3.4 release (catkin_generate_changelog, catkin_tag_changelog)
* Merge pull request #150 from ros-simulation/spawn_model_pose_fix
  Spawn model pose fix
* remove debug statement
* fix sdf spawn with initial pose
* fix sdf spawn with initial pose
* Merge pull request #148 from ros-simulation/spawn_model_pose_fix
  fix spawn initial pose.  When model has a non-zero initial pose and user...
* Merge branch 'hydro-devel' into spawn_model_pose_fix
* Merge pull request #149 from ros-simulation/fix_indentation
  fix indentation
* fix indentation
* Merge pull request #142 from hsu/hydro-devel
  fix issue #38, gui segfault on model deletion
* Merge pull request #140 from v4hn/spawn_model_sleep
  replace time.sleep by rospy.Rate.sleep
* Merge pull request #137 from fsuarez6/patch-1
  Add time import
* Merge pull request #132 from po1/fix-iterators
  Fix iterator-related things
* fix spawn initial pose.  When model has a non-zero initial pose and user specified initial model spawn pose, add the two.
* fix issue #38, gui segfault on model deletion by removing an obsolete call to set selected object state to "normal".
* replace time.sleep by rospy.Rate.sleep
  time was not even imported, so I don't know
  why this could ever have worked...
* Add time import
  When using the -wait option the script fails because is missing the time import
* Use pre-increment for iterators
* Fix iterator erase() problems
* Contributors: Francisco, John Hsu, Paul Mathieu, hsu, v4hn

2.3.3 (2013-10-10)
------------------
* "2.3.3"
* preparing for 2.3.3 release (catkin_generate_changelog, catkin_tag_changelog)
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Merge pull request #118 from ros-simulation/hydro-debug-cleanup
  Hydro debug cleanup
* Cleaned up unnecessary debug output that was recently added
* Merge pull request #116 from ros-simulation/hydro-catkin-fix
  Fix for multiple plugin install locations
* Fixed issue where catkin_find returns more than one library if it is installed from both source and debian
* Fixed issue where catkin_find returns more than one library if it is installed from both source and debian
* Contributors: Dave Coleman, Jim Rothrock, John Hsu, Nate Koenig

2.3.2 (2013-09-19)
------------------
* preparing for 2.3.2 release
* Merge pull request #114 from hsu/hydro-devel
  preparing for 2.3.2 release
* bump versions to 2.3.2
* Updating changelog for 2.3.2
* Merge pull request #104 from ros-simulation/synchronize_with_drcsim_plugins
  synchronize with drcsim plugins
* Merge pull request #108 from ros-simulation/fix_gazebo_includes
  Make gazebo includes use full path
* Make gazebo includes use full path
  In the next release of gazebo, it will be required to use the
  full path for include files. For example,
  include <physics/physics.hh> will not be valid
  include <gazebo/physics/physics.hh> must be done instead.
* update gazebo includes
* Merge branch 'hydro-devel' of github.com:ros-simulation/gazebo_ros_pkgs into synchronize_with_drcsim_plugins
* Merge pull request #106 from ericperko/hydro-devel
  gazebo_ros: Fixed a minor typo in spawn_model error message when -model not specified
* Fixed a minor typo in spawn_model error message when -model not specified
* Merge branch 'hydro-devel' into synchronize_with_drcsim_plugins
* Contributors: Eric Perko, John Hsu, Steven Peters, hsu

2.3.1 (2013-08-27)
------------------
* Updating changelogs
* Merge pull request #103 from ros-simulation/ros_control_plugin_header
  Created a header file for the ros_control gazebo plugin
* Cleaned up template, fixes for header files
* Contributors: Dave Coleman, William Woodall

2.3.0 (2013-08-12)
------------------
* Updated changelogs
* Merge branch 'hydro-devel' of https://github.com/ros-simulation/gazebo_ros_pkgs into hydro-devel
* Merge pull request #100 from ros-simulation/fix_osx
  Fixes found while building on OS X
* gazebo_ros: fixed missing dependency on TinyXML
* gazebo_plugins: replace deprecated boost function
  This is related to this gazebo issue:
  https://bitbucket.org/osrf/gazebo/issue/581/boost-shared\_-_cast-are-deprecated-removed
* Contributors: Dave Coleman, Piyush Khandelwal, William Woodall

2.2.1 (2013-07-29 18:02)
------------------------
* Updated changelogs
* Contributors: Dave Coleman

2.2.0 (2013-07-29 13:55)
------------------------
* Updated changelogs
* Switched to pcl_conversions
* Merged hydro branch
* Merge branch 'hydro-devel' into add_video_plugin
* Merged hydro-devel
* Merge pull request #87 from ros-simulation/remove_SDF_find_package_hydro
  Remove find_package(SDF) from CMakeLists.txt
* Remove find_package(SDF) from CMakeLists.txt
  It is sufficient to find gazebo, which will export the information
  about the SDFormat package.
* Merge branch 'tranmission_parsing' into groovy-devel
* Merge branch 'hydro-devel' into tranmission_parsing
* Merge branch 'hydro-devel' into merge_hydro_into_groovy
* Merge branch 'hydro-devel' into groovy-devel
* Merged hydro-devel branch in groovy-devel
* Merged hydro-devel
* Merged from Hydro-devel
* Merge branch 'hydro-devel' into tranmission_parsing
* Contributors: Dave Coleman, John Hsu, Piyush Khandelwal, Steven Peters

2.1.5 (2013-07-18)
------------------
* changelogs for 2.1.5
* Merge pull request #77 from meyerj/fix_gazebo_ros_paths_plugin_variable_names
  gazebo_ros: fixed variable names in gazebo_ros_paths_plugin
* gazebo_ros: fixed variable names in gazebo_ros_paths_plugin
* Contributors: Dave Coleman, Johannes Meyer, Tully Foote

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
* Contributors: Tully Foote

2.1.2 (2013-07-12)
------------------
* Added changelogs
* Added author
* Merge pull request #70 from ros-simulation/cmake_cleanup
  Cmake cleanup
* Tweak to make SDFConfig.cmake
* Merge pull request #69 from ros-simulation/dev
  Cleaned up gazebo_ros_paths_plugin
* Cleaned up CMakeLists.txt for all gazebo_ros_pkgs
* Cleaned up gazebo_ros_paths_plugin
* Contributors: Dave Coleman, hsu

2.1.1 (2013-07-10)
------------------
* Merge branch 'hydro-devel' of github.com:ros-simulation/gazebo_ros_pkgs into hydro-devel
* Reduced number of debug msgs
* Merge pull request #66 from ros-simulation/dynamic_reconfigure
  Fixed dynamic reconfigure namespace, cleaned up various code
* Fixed physics dynamic reconfigure namespace
* Merge branch 'hydro-devel' into dev
* Merge pull request #65 from meyerj/fix_gazebo_ros_api_plugin_loaded_flag
  gazebo_ros: GazeboRosApiPlugin is not properly unloaded during destruction
* gazebo_ros_api_plugin: set plugin_loaded\_ flag to true in
  GazeboRosApiPlugin::Load() function
* Merge pull request #59 from ros-simulation/CMake_Tweak
  Added dependency to prevent missing msg header, cleaned up CMakeLists
* Merge pull request #62 from ros-simulation/move_python_pkgs
  Moved gazebo_interface.py from gazebo/ folder to gazebo_ros/ folder
* Merge pull request #61 from ros-simulation/no_gazebo_pkg
  No gazebo pkg
* Merge branch 'move_python_pkgs' into dev
* Actually we need __init_\_.py
* Cleaning up code
* Merge branch 'no_gazebo_pkg' into dev
* Merge branch 'move_python_pkgs' into dev
* Merge branch 'CMake_Tweak' into dev
* Moved gazebo_interface.py from gazebo/ folder to gazebo_ros/ folder
* Removed searching for plugins under 'gazebo' pkg because of rospack warnings
* Minor print modification
* Added dependency to prevent missing msg header, cleaned up CMakeLists
* Contributors: Dave Coleman, Johannes Meyer

2.1.0 (2013-06-27)
------------------
* Merge pull request #34 from meyerj/support_gazebo_package_name_for_plugins_patch
  also support gazebo instead of gazebo_ros for package exports
* gazebo_ros: added deprecated warning for packages that use gazebo as
  package name for exported paths
* Merge branch 'hydro-devel' of github.com:osrf/gazebo_ros_pkgs into hydro-devel
* Hiding some debug info
* Merge pull request #49 from meyerj/gazebo_ros_debug_install_space_fix
  debug script does not work in install space
* gazebo_ros: use rosrun in debug script, as rospack find gazebo_ros returns the wrong path in install space
* Hide Model XML debut output to console
* Merge remote-tracking branch 'origin/hydro-devel' into robot_hw_sim
* Merge pull request #42 from osrf/api_plugin_no_include
  gazebo_ros_api_plugin.h is no longer exposed in the include folder
* Merge branch 'hydro-devel' of https://github.com/osrf/gazebo_ros_pkgs into terminate_service_thread_fix
  Conflicts:
  gazebo_plugins/include/gazebo_plugins/PubQueue.h
* gazebo_ros_api_plugin.h is no longer exposed in the include folder
* Merge pull request #35 from meyerj/fix_include_directory_installation_target
  Header files of packages gazebo_ros and gazebo_plugins are installed to the wrong location
* Added args to launch files, documentation
* Merge pull request #28 from osrf/no_roscore_handling
  Better handling of gazebo_ros run when no roscore started
* gazebo_ros: also support gazebo instead of gazebo_ros as package name for plugin_path, gazebo_model_path or gazebo_media_path exports
* gazebo_plugins/gazebo_ros: fixed install directories for include files and gazebo scripts
* Merge pull request #26 from piyushk/robot-namespace-fix
  SDF and URDF now set robotNamespace for plugins
* changed comment location
* added block comments for walkChildAddRobotNamespace
* SDF and URDF now set robotNamespace for plugins
* Better handling of gazebo_ros run when no roscore started
* Contributors: Dave Coleman, Johannes Meyer, Piyush Khandelwal

2.0.2 (2013-06-20)
------------------
* Added Gazebo dependency
* Merge pull request #19 from piyushk/gazebo-script-bash-fix
  modified script to work in bash correctly (tested on ubuntu 12.04 LTS)
* changed the final kill to send a SIGINT and ensure only the last background process is killed.
* modified script to work in bash correctly (tested on ubuntu 12.04 LTS)
* Contributors: Dave Coleman, Piyush Khandelwal

2.0.1 (2013-06-19)
------------------
* Incremented version to 2.0.1
* Fixed circular dependency, removed deprecated pkgs since its a stand alone pkg
* Merge branch 'dave_dev' into hydro-devel
* Shortened line lengths of function headers
* Contributors: Dave Coleman

2.0.0 (2013-06-18)
------------------
* Changed version to 2.0.0 based on gazebo_simulator being 1.0.0
* Updated package.xml files for ros.org documentation purposes
* Merge pull request #15 from osrf/topics_services
  Revamped Gazebo Services
* Combined updateSDFModelPose and updateSDFName, added ability to spawn SDFs from model database, updates SDF version to lastest in parts of code, updated the tests
* Renamed Gazebo model to SDF model, added ability to spawn from online database
* Merge pull request #11 from osrf/plugin_updates
  Merged Atlas ROS Plugins
* Fixed really obvious error checking bug
* Deprecated -gazebo arg in favor of -sdf tag
* Reordered services and messages to be organized and reflect documentation. No code change
* Cleaned up file, addded debug info
* Merged changes from Atlas ROS plugins, cleaned up headers
* Merge pull request #8 from osrf/code_cleanup
  Code cleanup
* Small fixes per ffurrer's code review
* Deprecated warnings fixes
* Cleaned up comment blocks - removed from .cpp and added to .h
* Merged branches and more small cleanups
* Merge pull request #5 from osrf/shutdown_segfault_fix
  Shutdown segfault fix
* Small compile error fix
* Standardized function and variable naming convention, cleaned up function comments
* Reduced debug output and refresh frequency of robot spawner
* Converted all non-Gazebo pointers to boost shared_ptrs
* Removed old Gazebo XML handling functions - has been replaced by SDF, various code cleanup
* Removed the physics reconfigure node handle, switched to async ROS spinner, reduced required while loops
* Merge branch 'groovy-devel' of github.com:osrf/gazebo_pkgs into shutdown_segfault_fix
* Fixed shutdown segfault, renamed rosnode\_ to nh\_, made all member variables have _ at end, formatted functions
* Added small comment
* Merge branch 'groovy-devel' of https://github.com/osrf/gazebo_pkgs into groovy-devel
* adding install for gazebo_ros launchfiles
* Merge branch 'groovy-devel' into shutdown_segfault_fix
* Merge pull request #4 from osrf/ros_formatting
  Formatted files to be double space indent per ROS standards
* Formatted files to be double space indent per ROS standards
* Started fixing thread issues
* Merge pull request #3 from jhu-lcsr-forks/groovy-devel
  Fixing install script names
* Fixing install script names and adding gzserver and gdbrun to install command
* Fixed deprecated warnings, auto formatted file
* Cleaned up status messages
* Added -h -help --help arguemnts to spawn_model
* Merge branch 'groovy-devel' of github.com:osrf/gazebo_pkgs into groovy-devel
* Removed broken worlds
* Removed deprecated namespace argument
* Merge pull request #1 from fmder/groovy-devel
  Path to setup.sh was hard coded in the scripts
* Using pkg-config to find the script installation path.
  Corrected a bash typo with client_final variable in gazebo script.
* Cleaning up world files
* Deprecated fix
* Moved from gazebo_worlds
* Cleaning up launch files
* Moved from gazebo_worlds
* Fixing renaming errors
* Updated launch and world files and moved to gazebo_ros
* Combined gzclient and gzserver
* Added finished loading msg
* All packages building in Groovy/Catkin
* Imported from bitbucket.org
* Contributors: Dave Coleman, Jonathan Bohren, fmder1, hsu
