^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_plugin_covariance
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.6 (2018-01-09)
------------------
* Merge branch 'qt5-compatibility' into 'cobalt-devel'
  Make it qt5 compatible
  See merge request navigation/rviz_plugin_covariance!3
* Make it qt5 compatible
* Contributors: Jeremie Deray, Victor Lopez

0.0.5 (2016-03-02)
------------------
* cmake_modules
* Contributors: Jeremie Deray

0.0.4 (2015-02-20)
------------------
* Merge branch 'properties_fix' into 'cobalt-devel'
  Properties Fix
  Sorry if you think something is duplicated, but it works like this.
* Fix applying of the properties of the plugin (axes mainly)
* Contributors: Enrique Fernandez, Sammy Pfeiffer

0.0.3 (2015-02-19)
------------------
* Add show position property
* Add show orientation property
* Fix show axis
* Fix orientation for 3DOF
* Check for eigensolver status
* Contributors: Enrique Fernandez

0.0.2 (2015-02-19)
------------------
* Add 3DOF visual
* Catkinize and update rviz plugin API
* Merge pull request #4 from flixr/find_eigen
  remove FindEigen cmake script, so the system script is used
* remove FindEigen cmake script, so the system script is used
* Merge pull request #3 from flixr/fix_crash_on_nan
  Prevent Rviz from crashing due to NaNs.
* Don't set position/orientation/scale if they contain NaNs as this will cause Rviz to crash.
  This is a very crude method, instead of just not setting the respective values,
  the markers should probably not be displayed at all (or something like that...)
* Merge pull request #2 from flixr/fix_ellipsoid_orientation
  Partial fix for the covariance ellipsoid orientation.
* partially fix orientation of covariance ellipsoid
  * do not normalize matrix with eigen vectors as the eigen vectors (columns) are already normalized
* Fix bugs, add test cases.
* Add odometry support.
* Fix matrice sizes and recopy.
* Update README.
* Add README.
* Improve code, add scaling factor.
* Initial import.
* Contributors: Enrique Fernandez, Felix Ruess, Thomas Moulard
