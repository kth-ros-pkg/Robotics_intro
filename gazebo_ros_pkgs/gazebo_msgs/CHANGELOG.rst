^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.7 (2018-01-12)
------------------

2.6.6 (2018-01-12)
------------------

2.6.5 (2018-01-11)
------------------
* removed changelogs and unified package versions
* Contributors: Hilario Tome

2.5.14 (2017-12-11)
-------------------
* Generate changelogs
* Contributors: Jose Luis Rivero

2.5.13 (2017-06-24)
-------------------
* Update changelogs
* Add catkin package(s) to provide the default version of Gazebo - take II (kinetic-devel) (#571)
  * Added catkin package gazebo_dev which provides the cmake config of the installed Gazebo version
  Conflicts:
  gazebo_plugins/package.xml
  gazebo_ros/package.xml
  gazebo_ros_control/package.xml
  * gazebo_plugins/gazebo_ros: removed dependency SDF from CMakeLists.txt
  The sdformat library is an indirect dependency of Gazebo and does not need to be linked explicitly.
  * gazebo_dev: added execution dependency gazebo
* Contributors: Jose Luis Rivero

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
* Contributors: Alessandro Ambrosano, Jose Luis Rivero

2.5.10 (2017-03-03)
-------------------
* Changelogs for 2.5.10
* Contributors: Jose Luis Rivero

2.5.9 (2017-02-20)
------------------
* Update changelogs
* Merge pull request #539 from davetcoleman/kinetic-whitespace
  Removed all trailing whitespace
* Removed all trailing whitespace
* Contributors: Dave Coleman, Jose Luis Rivero

2.5.8 (2016-12-06)
------------------
* Update changelogs for 2.5.8
* Contributors: Jose Luis Rivero

2.5.7 (2016-06-10)
------------------
* Update changelogs
* Contributors: Jose Luis Rivero

2.5.6 (2016-04-28)
------------------
* Fix versions in CHANGELOG
* 2.5.5
* Update changelogs
* Contributors: Jose Luis Rivero

2.5.4 (2016-04-27)
------------------
* Update changelogs
* Merge pull request #454 from scpeters/merge_ijk
  merge indigo, jade to kinetic-devel
* merge indigo, jade to kinetic-devel
* Merge pull request #430 from ros-simulation/kinetic-devel-maintainer
  Update maintainer for Kinetic release
* Update maintainer for Kinetic release
* [gazebo_msgs] fix wrong dependencies
* Contributors: Jose Luis Rivero, Steven Peters, Yuki Furuta

2.5.3 (2016-04-11)
------------------
* Update changelogs for 2.5.3
* Merge branch 'jade-devel' into issue_387_remove_ros_remappings
* Contributors: Jose Luis Rivero, Martin Pecka

2.5.2 (2016-02-25)
------------------
* Fix changelog in gazebo_msgs
* Prepare changelogs
* merging from indigo-devel
* Merge pull request #302 from maxbader/jade-devel-GetModelState
  Header for GetModelState service request for jade-devel
* 2.4.9
* Generate changelog
* Merge pull request #2 from ros-simulation/indigo-devel
  Indigo devel
* GetModelState modification for jade
* Contributors: John Hsu, Jose Luis Rivero, Markus Bader, hsu, iche033

2.5.1 (2015-08-16 02:31)
------------------------
* Generate changelogs
* Contributors: Jose Luis Rivero

2.5.0 (2015-04-30)
------------------
* changelogs
* Contributors: William Woodall

2.4.9 (2015-08-16 01:30)
------------------------
* Generate changelog
* Merge pull request #2 from ros-simulation/indigo-devel
  Indigo devel
* Contributors: Jose Luis Rivero, iche033

2.4.8 (2015-03-17)
------------------
* Generate new changelog
* Contributors: Jose Luis Rivero

2.4.7 (2014-12-15)
------------------
* Changelogs for 2.4.7 branch
* Merge pull request #255 from ros-simulation/fix_gazebo_ros_tutorial_url
  Update Gazebo/ROS tutorial URL
* Update Gazebo/ROS tutorial URL
* Contributors: Jose Luis Rivero

2.4.6 (2014-09-01)
------------------
* Changelogs for version 2.4.6
* 2.3.6
* Update changelogs for the upcoming release
* Merge remote-tracking branch 'origin/hydro-devel' into camera-info-manager
* Merge pull request #1 from ros-simulation/hydro-devel
  Merge from upstream
* Contributors: Jonathan Bohren, Jose Luis Rivero

2.4.5 (2014-08-18)
------------------
* Changelogs for upcoming release
* Contributors: Jose Luis Rivero

2.4.4 (2014-07-18)
------------------
* Update Changelog
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Merge pull request #201 from jonbinney/indigo-repos
  Fix repository urls for indigo branch
* Merge pull request #202 from jonbinney/hydro-repos
  Fix repo names in package.xml's (hydro-devel branch)
* Fix repo names in package.xml's
* Fix repo names in package.xml's
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Merge pull request #1 from ros-simulation/hydro-devel
  Merge from upstream
* Contributors: Jon Binney, Jonathan Bohren, Markus Bader, Steven Peters

2.4.3 (2014-05-12)
------------------
* update changelog
* Contributors: Steven Peters

2.4.2 (2014-03-27)
------------------
* catkin_tag_changelog
* catkin_generate_changelog
* merging from hydro-devel
* 2.3.5
* catkin_tag_changelog
* catkin_generate_changelog and fix rst format for forthcoming logs
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Contributors: Jim Rothrock, John Hsu

2.4.1 (2013-11-13 18:52)
------------------------
* bump patch version for indigo-devel to 2.4.1
* merging from indigo-devel after 2.3.4 release
* "2.3.4"
* preparing for 2.3.4 release (catkin_generate_changelog, catkin_tag_changelog)
* Contributors: John Hsu

2.4.0 (2013-10-14)
------------------
* "2.4.0"
* catkin_generate_changelog
* Contributors: John Hsu

2.3.5 (2014-03-26)
------------------
* catkin_tag_changelog
* catkin_generate_changelog and fix rst format for forthcoming logs
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Contributors: Jim Rothrock, John Hsu

2.3.4 (2013-11-13 18:05)
------------------------
* "2.3.4"
* preparing for 2.3.4 release (catkin_generate_changelog, catkin_tag_changelog)
* Contributors: John Hsu

2.3.3 (2013-10-10)
------------------
* "2.3.3"
* preparing for 2.3.3 release (catkin_generate_changelog, catkin_tag_changelog)
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* Contributors: Jim Rothrock, John Hsu

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
* Contributors: William Woodall

2.3.0 (2013-08-12)
------------------
* Updated changelogs
* Contributors: Dave Coleman

2.2.1 (2013-07-29 18:02)
------------------------
* Updated changelogs
* Contributors: Dave Coleman

2.2.0 (2013-07-29 13:55)
------------------------
* Updated changelogs
* Merge branch 'tranmission_parsing' into groovy-devel
* Merge branch 'hydro-devel' into tranmission_parsing
* Merge branch 'hydro-devel' into merge_hydro_into_groovy
* Merged hydro-devel branch in groovy-devel
* Merged hydro-devel
* Merged from Hydro-devel
* Contributors: Dave Coleman, John Hsu

2.1.5 (2013-07-18)
------------------
* changelogs for 2.1.5
* Contributors: Tully Foote

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
* Merge pull request #70 from ros-simulation/cmake_cleanup
  Cmake cleanup
* Cleaned up CMakeLists.txt for all gazebo_ros_pkgs
* Contributors: Dave Coleman

2.1.1 (2013-07-10)
------------------

2.1.0 (2013-06-27)
------------------

2.0.2 (2013-06-20)
------------------

2.0.1 (2013-06-19)
------------------
* Incremented version to 2.0.1
* Contributors: Dave Coleman

2.0.0 (2013-06-18)
------------------
* Changed version to 2.0.0 based on gazebo_simulator being 1.0.0
* Updated package.xml files for ros.org documentation purposes
* Imported from bitbucket.org
* Contributors: Dave Coleman
