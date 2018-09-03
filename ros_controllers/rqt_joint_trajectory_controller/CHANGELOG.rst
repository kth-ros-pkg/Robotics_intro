^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_joint_trajectory_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge branch 'as_dynamic_trajectory' into 'erbium-devel'
  pal_joint_trajectory_controller + fixes in rqt_joint_trajectory_controller
  See merge request control/ros_controllers!35
* pal_joint_trajectory_controller + fixes in rqt_joint_trajectory_controller
* Contributors: Hilario Tome, alexandersherikov

0.3.8 (2018-05-08)
------------------

0.3.9 (2018-05-29)
------------------

0.3.10 (2018-06-15)
-------------------

0.3.11 (2018-06-19)
-------------------
* Merge branch 'as_rqt_jtc_fix_uncontrolled_joint' into 'erbium-devel'
  rqt_joint_trajectory_controller: allow partial control
  See merge request control/ros_controllers!39
* rqt_joint_trajectory_controller: allow partial control
* Contributors: Hilario Tome, alexandersherikov

0.13.1 (2017-11-06)
-------------------
* Update changelogs
* Contributors: Bence Magyar

0.13.0 (2017-08-10)
-------------------
* Update changelogs
* Contributors: Bence Magyar

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
* Contributors: Bence Magyar

0.12.0 (2017-02-15)
-------------------
* Update changelogs
* Merge pull request #242 from bmagyar/update_package_xmls
  Update package xmls
* Change for format2
* Add Enrique and Bence to maintainers
* Merge branch 'kinetic-devel' into F_enable_part_traj_kinetic
* Contributors: Bence Magyar, beatrizleon

0.11.2 (2016-08-16)
-------------------
* Update changelogs
* Merge pull request #231 from beta-robots/qt5_fixes
  Changes in import of Qt modules
* Changes in import of Qt modules
  RQt supports Qt5 in the Kinetic release (see the migration guide
  http://wiki.ros.org/kinetic/Migration). In Qt5, QWidget and QFormLayout are
  found in QtWidgets, see this ROS answer:
  http://answers.ros.org/question/235126/import-issues-in-ros-kinetic-rqt/
  Fixes #230
* Contributors: Bence Magyar, Carlos J. Rosales Gallegos

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
* rqt_jtc: Migrate to ROS jade
  Adapt to new controller_manager_msgs/ControllerState message definition
* Merge pull request #189 from ros-controls/rqt-jtc-improvements
  [rqt_joint_trajectory_controller] Many small improvements
* Merge pull request #193 from ros-controls/rqt-jtc-vscroll
  rqt_jtc: Add vertical scrollbar to joints list
* Merge pull request #192 from ros-controls/rqt-jtc-clear-ctrls
  rqt_jtc: Clear controllers combo on cm change
* rqt_jtc: Add vertical scrollbar to joints list
  - Add vertical scrollbar to joints list that appears only when required,
  i.e., when the plugin size cannot accommodate all controller joints.
  - Remove vertical spacer at the bottom of the plugin.
* rqt_jtc: Clear controllers combo on cm change
  Clear the list of running joint trajectory controllers when the
  controller manager selection changes. This prevents potential conflicts when
  multiple controller managers have controllers with the same names.
* rqt_jtc: Fail gracefully if URDF is not loaded
  Fixes #179.
  Implement lazy loading of joint limits from URDF.
  Since the JointTrajectoryController parses the URDF to determine if joints are
  continuous or not, having at least one running controller means that the URDF
  is loaded in the ROS parameter server. We thus defer joint limits parsing to
  when we know there is at least one running controller.
  This allows to start rqt_joint_trajectory_controller on an otherwise empty ROS
  node graph without crashing.
* rqt_jtc: Save and restore plugin settings
  Fixes #188.
  - Save current controller_manager and controller selection on plugin close
  - Restore last selection if controller manager and controller are running
* rqt_jtc: Stricter controller validation
  Only display in the controller combo box those controllers that are running
  _and\_ have position and velocity limits specified in the URDF. In the absence
  of limits information, it's not posible to properly initialize the GUI sliders.
* rqt_jtc: Fix broken URDF joint limits parsing
* rqt_jtc: Add controller resources query
  Factor in a common method how controller resources are queried. This function,
  which is currently a one-liner, will be reimplemented in ROS jade, as the
  controller_manager_msgs/ControllerState message has changed.
* rqt_jtc: Don't choke on missing URDF vel limits
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.2 (2015-05-04)
------------------
* Update changelogs.
* Merge pull request #156 from ros-controls/rqt-jtc-dep
  rqt_joint_traj_controller: Add missing runtime dep
* rqt_joint_traj_controller: Add missing runtime dep
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.1 (2014-11-03)
------------------
* Update changelogs
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.0 (2014-10-31)
------------------
* Update changelogs
* Sync version with rest of ros_controllers
* Merge pull request #121 from pal-robotics/rqt-jtc
  Fix initial sync of DoubleEditor widgets
* Fix initial sync of DoubleEditor widgets
* Merge pull request #103 from pal-robotics/rqt-jtc
  joint_trajectory_controller rqt plugin
* Create a joint_trajectory_controller rqt plugin.
  Initial plugin features:
  - Allows to select any _running\_ joint trajectory controller from any active
  controller manager.
  - Two modes:
  - Monitor: Joint display shows actual positions of controller joints
  - Control: Joint display sends controller commands
  - Max joint speed is read from the URDF, but can be scaled down for safety.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.8.1 (2014-07-11)
------------------

0.8.0 (2014-05-12)
------------------

0.7.2 (2014-04-01)
------------------

0.7.1 (2014-03-31)
------------------

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
