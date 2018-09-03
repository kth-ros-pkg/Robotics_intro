^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_interaction_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Added action definition for file-based speech recognition
* Contributors: Jordi Adell

0.10.1 (2014-11-17)
-------------------
* Actions and messages moved to CamelCase
* New ttsi18n.action and ttstext.action
  New TTS action server that supports internationalisation
  as defined in pal_tts_cfg.  ttstext.action is aimed at replaced
  the old Sound.action API which is deprecated now.
* Contributors: Jordi Adell

0.9.1 (2014-05-27)
------------------
* Added a calibration request
  Aslo changed some comments as documentation
* Corrected some comments to match the code
* Typo corrected and new events added
  I corrected a typo in listen_state variable
  and added the following events:
  FAILED_DECODING
  CALIBRATION
* Grammar manament message removed.
* Add the new grammar management message CMakeLists
  Also changed the definition of ASRSrvRequest to deal with this new
  general definition.
* Added a new definition for general language model management
  This can be used for grammars, ngrams, and keyword spotting.
  This will replace current grammar management definition.
* pal_interaction_msgs: re-add Sound action
* Fix uint8 -> int8
* Fix minor typos and 0 codes
* pal_interaction_msgs: list all new files in CMakeLists.txt
* pal_interaction_msgs: sync with svn
  Some messages have been lost in the migration, restoring them.
* pal_interaction_msgs: fix message generation
* Added other packages needed by people that want to use our robot, face
  detection in pal_detection_msgs, and text to speech in text_to_speech. Also
  removed from pal_interaction_msgs the references to the speech part that was
  included there and made incompatible the use of axclient without having the
  same package name than the one inside of the real robot
* Add pal_interaction_msgs and metapackage
* Contributors: Jordi Adell, Paul Mathieu, Sammy Pfeiffer
