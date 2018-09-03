^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_controller_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge pull request #254 from bmagyar/update_package_xmls
  Update package xmls
* Add Enrique and Bence to maintainer list
* Convert to format2, fix dependency in cmake
* 0.11.1
* Update changelog
* Merge pull request #249 from bmagyar/qt5-fix
  Qt5 migration
* Qt5 migration
* 0.11.0
* Update changelogs
* 0.10.1
* Update changelogs
* 0.10.0
* Update changelogs
* Merge pull request #204 from ros-controls/multi-iface-ctrl
  Allow controllers to claim resources from multiple interfaces
* rqt_cm: Allow running as standalone application
* rqt_cm: Multi-interface controllers, UI revamp
  - Make the rqt_controller_manager aware of multi-interface controllers.
  - Reduce the amount of screen real-estate used by the plugin. Controller
  information that was previously shown in the plugin widget has been
  deferred to a pop-up that appears when double-clicking a controller.
  - Make main view read-only. To affect the state of available controllers
  (load, unload,start, stop), one must take explicit action: right-click
  context menu. This was already so to a certain exxtent, but you could
  still load and start uninitialized controllers from the main view.
  - Show uninitialized controllers (fetched from parameter server) in the same
  list as stopped and running controllers.
  - Make less assumptions when finding running controller managers. Use
  existing helpers on controller_mamager_msgs.utils Python module.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Bence Magyar, Hilario Tome, Sam Pfeiffer, Victor Lopez

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
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.8.2 (2014-06-25)
------------------
* Update changelogs
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.8.1 (2014-06-24)
------------------
* Update changelogs.
* Merge pull request #163 from pal-robotics/rqt-cm-group
  Register plugin under a group. Fixes #162.
* Register plugin under a group. Fixes #162.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.8.0 (2014-05-12)
------------------
* Updated changelogs
* Contributors: Dave Coleman

0.7.2 (2014-04-01)
------------------
* Prepare 0.7.2
* Merge pull request #150 from ros-controls/rqt-cm-install-resources
  Add plugin resources to installation target.
* Add plugin resources to installation target.
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
* Cleanedup changelog
* Updated changelogs
* Updated changelogs
* Merge pull request #132 from kphawkins/hydro-devel-rqt-plugin
  Initial release for a rqt controller manager plugin
* Added controller namespace detection and switching, loadable controller parameter detection and buttons for loading or starting the controller directly from the parameter server.
* Resources -> Claimed Resources column title
* Initial commit for rqt controller manager plugin.  Plugin seems functional from first tests.  Allows users to unload/load/start/stop/view available controllers.  No functionality yet exists for loading a controller from scratch.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, Kelsey

0.5.8 (2013-10-11)
------------------

0.5.7 (2013-07-30)
------------------

0.5.6 (2013-07-29)
------------------

0.5.5 (2013-07-23 17:04)
------------------------

0.5.4 (2013-07-23 14:37)
------------------------

0.5.3 (2013-07-22 18:06)
------------------------

0.5.2 (2013-07-22 15:00)
------------------------

0.5.1 (2013-07-19)
------------------

0.5.0 (2013-07-16)
------------------

0.4.0 (2013-06-25)
------------------
