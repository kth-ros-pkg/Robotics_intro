^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.2 (2016-09-19)
-------------------

0.11.1 (2016-07-11)
-------------------

0.11.0 (2016-07-11)
-------------------

0.10.4 (2015-09-04)
-------------------

0.10.3 (2015-03-09)
-------------------

0.10.2 (2015-02-06)
-------------------

0.10.1 (2014-11-17)
-------------------
* Fix email address
* Contributors: Víctor López

0.9.1 (2014-05-27)
------------------
* Update meta-package with pal_walking_msgs and pal_navigation_msgs
* Remove unneeded package text_to_speech
  Its functionalities (Sound action) have been merged
  into pal_interaction_msgs in the previous commit.
* Update metapackage
* Add pal_device_msgs
* Added other packages needed by people that want to use our robot, face
  detection in pal_detection_msgs, and text to speech in text_to_speech. Also
  removed from pal_interaction_msgs the references to the speech part that was
  included there and made incompatible the use of axclient without having the
  same package name than the one inside of the real robot
* Add pal_interaction_msgs and metapackage
* Contributors: Paul Mathieu, Sammy Pfeiffer
