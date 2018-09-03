^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_detection_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.2 (2016-09-19)
-------------------

0.11.1 (2016-07-11)
-------------------

0.11.0 (2016-07-11)
-------------------
* add emotions fields
* Contributors: Jordi Pages

0.10.4 (2015-09-04)
-------------------
* add facial expressions and confidence information
* Contributors: Jordi Pages

0.10.3 (2015-03-09)
-------------------

0.10.2 (2015-02-06)
-------------------

0.10.1 (2014-11-17)
-------------------

0.9.1 (2014-05-27)
------------------
* Modifications for the pal_person_detection_fuser
* add size of object in SelectTexturedObject
  and add comments on both messages
* add services for textured object detection
* image included in the message is now compressed
* pal_detection_msgs: fix message generation
* Added other packages needed by people that want to use our robot, face
  detection in pal_detection_msgs, and text to speech in text_to_speech. Also
  removed from pal_interaction_msgs the references to the speech part that was
  included there and made incompatible the use of axclient without having the
  same package name than the one inside of the real robot
* Contributors: Jordi Pages, Paul Mathieu, Sammy Pfeiffer
