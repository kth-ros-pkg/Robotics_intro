^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamic_introspection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.7 (2018-06-08)
------------------
* Merge branch 'online_write_bag' into 'erbium-devel'
  added dumping bag
  See merge request control/dynamic_introspection!12
* added dumping bag
* Contributors: Hilario Tome

1.0.6 (2018-06-07)
------------------
* Merge branch 'online_utils' into 'erbium-devel'
  added online utils
  See merge request control/dynamic_introspection!11
* reenabled get number of messages
* added online utils
* Merge branch 'more_helpers' into 'erbium-devel'
  cleanup of matlab helpers
  See merge request control/dynamic_introspection!10
* cleanup of matlab helpers
* Merge branch 'removed_pal_utils' into 'erbium-devel'
  removed pal utils dependency
  See merge request control/dynamic_introspection!9
* removed pal utils dependency
* Merge branch 'extra_asserts_utils' into 'erbium-devel'
  added extra asserts introspection utils
  See merge request control/dynamic_introspection!8
* added extra asserts introspection utils
* Add dependency for singleton_test
* Contributors: Hilario Tome, Victor Lopez

1.0.5 (2018-04-25)
------------------
* Merge branch 'ndebug' into 'erbium-devel'
  ignore exception if ndegug set
  See merge request control/dynamic_introspection!7
* Enforce add exported targets dependencies
* added missing ndegug
* added verbose in case ndebug set
* ignore exception if ndegug set
* Contributors: Hilario Tome, Victor Lopez

1.0.4 (2018-04-18)
------------------
* Merge branch 'find-eigen3' into 'erbium-devel'
  Find Eigen3
  See merge request control/dynamic_introspection!6
* Find Eigen3
  The find we're using is in cmake_modules and is deprecated
* Merge branch 'removed_pal_utils' into 'erbium-devel'
  removed pal_utils
  See merge request control/dynamic_introspection!5
* removed pal_utils
* Contributors: Hilario Tome, Victor Lopez

1.0.3 (2018-04-09)
------------------
* Merge branch 'more_helpers' into 'erbium-devel'
  added more helper utils
  See merge request control/dynamic_introspection!4
* added more helpers
* added more helper utils
* 1.0.2
* updated changelog
* Merge branch 'concurrency' into 'erbium-devel'
  Concurrency
  See merge request control/dynamic_introspection!3
* added missing pal_utils depend
* fixed merge
* fixed utils
* added support for intermidient logged variables
* Merge branch 'add-stamp' into erbium-devel
* Add timestamp to DynamicIntrospection message
* Contributors: Hilario Tome, Victor Lopez

1.0.1 (2018-03-23)
------------------
* Merge branch 'header_stamp' into 'erbium-devel'
  added header stamp
  See merge request control/dynamic_introspection!2
* added header stamp
* Contributors: Hilario Tome

1.0.0 (2018-03-19)
------------------
* added install rules for matlab folder and removed deprecated matlab files
* Merge branch 'add-stamp' into 'erbium-devel'
  Add timestamp to DynamicIntrospection message
  See merge request control/dynamic_introspection!1
* Add timestamp to DynamicIntrospection message
* changed to package2
* Contributors: Hilario Tome, Victor Lopez

1.0.2 (2018-04-01)
------------------
* Merge branch 'concurrency' into 'erbium-devel'
  Concurrency
  See merge request control/dynamic_introspection!3
* added missing pal_utils depend
* fixed merge
* fixed utils
* 1.0.1
* updated changelog
* Merge branch 'header_stamp' into 'erbium-devel'
  added header stamp
  See merge request control/dynamic_introspection!2
* added header stamp
* 1.0.0
* Update changelog
* added install rules for matlab folder and removed deprecated matlab files
* added support for intermidient logged variables
* Merge branch 'add-stamp' into 'erbium-devel'
  Add timestamp to DynamicIntrospection message
  See merge request control/dynamic_introspection!1
* Add timestamp to DynamicIntrospection message
* Merge branch 'add-stamp' into erbium-devel
* changed to package2
* Add timestamp to DynamicIntrospection message
* Contributors: Hilario Tome, Victor Lopez

0.1.0 (2018-01-15)
------------------
* formating
* formating
* added bag reading topic parameter
* fixed introspection utils
* changed dynamic introspection execption to run_time exception
* added get number of subscribers
* clean up
* Fixed merge
* Fixed bug in as flag compilation error
* Contributors: Adria Roig, Hilario Tome, Hilario Tomé

0.0.6 (2017-02-17)
------------------
* Merge branch 'dubnium-devel' of gitlab:control/dynamic_introspection into dubnium-devel
* Updated changelog
* Fixed quaternion introspection
* Fixed cppcheck example
* removed comented code
* removed comented code
* Finished refactoring to make thread and realtime safe
* Contributors: Hilario Tome

* Fixed quaternion introspection
* removed comented code
* removed comented code
* Finished refactoring to make thread and realtime safe
* Contributors: Hilario Tome

0.0.5 (2016-11-09)
------------------
* Merge branch 'dubnium-devel' of gitlab:control/dynamic_introspection into dubnium-devel
* Changed introspection bag tools defaut topic
* Merge branch 'dubnium-devel' of gitlab:control/dynamic_introspection into dubnium-devel
* Started to implement dynamic introspection server
* Contributors: Hilario Tome

0.0.4 (2016-10-12)
------------------
* Added missing depend
* Added only basic type registering
* IntrospectionBagReader
* Added introspection bag reader
* Removed registering of vector and matrix, added markers registration
* Merge branch 'dubnium-devel' of gitlab:control/dynamic_introspection into dubnium-devel
* Added zmp debuggin matlab
* Added matrix3d registering and estimation matlab
* Contributors: Hilario Tome

0.0.3 (2016-03-09)
------------------
* Changed ros publisher to real time publisher
* removed eigen map registering
* Continue matlab developing
* continue matlab implementation
* Initial commit of matlab folder
* Added map3 registering
* Added unregister exception
* Removed loggin level
* Fixed bug
* Added exception throwing
* Added output topic configuration
* Added unregister macro
* Added todo
* Working dynamic intstrospection with plugins
* Added plugin cpp
* Working singleton with test exapmle of shared libraries
* Merge branch 'cobalt-devel' of gitlab:control/dynamic_introspection into cobalt-devel
* Added Eigen aligned operator
* Added singleton test examples
* Added Eigen 3d vector support (not compatible with Eigen::Dynamic)
* Contributors: Hilario Tome

0.0.2 (2015-06-10)
------------------
* Added license and documentation
* Contributors: Hilario Tome

0.0.1 (2015-05-26)
------------------
* Added install targets
* Added Eigen deps
* removed printing message from multiple plot
* Multiplot working for online trajectory generation
* Topic plotting working
* Added multiple figure plotting python script
* Added python sript to read test bag, and pydev project
* Added bag to dynamic introspection
* Initial commit
* Contributors: Hilario Tome
