# ASR result messages used by RosRecognizerServer
# This message are published in the ASR event topic.
# The kind of events that are published are the following ones:
## Event types to be published.
int8 EVENT_UNDEFINED          = -1
int8 EVENT_LISTEN_STATE       = 1
int8 EVENT_RECOGNIZED_UTT     = 2
int8 EVENT_ASR_ACTIVATION     = 3
int8 EVENT_FAILED_DECODING    = 4

# 1-> The ASR changed its listening state.
# 2-> The ASR recognized a full uterance and published the result
# 3-> The ASR has been activated/deactivated
# 4-> The ASR tried to regcognise an utterance unsuccesfuly

# These are the possible listening states published when event_id = LISTENING_STATE

## Listen states
int8 LISTEN_UNDEFINED = 20
int8 LISTEN_CALIBRATION = 21
int8 LISTEN_WAITING_FOR_SPEECH = 22
int8 LISTEN_SPEECH_DETECTED = 23
int8 LISTEN_END_OF_SPEECH = 24

# 21-> Audio calibration is taking place
# 22-> Some speech has been detected.
# 23-> The ASR is waiting for someone to start speaking.
# 24-> The end of an utterance has been found, and will start decoding it.

##  Message variables

# The event type published.
int8 event_id

# The listening state of the ASR
int8 listen_state

# The recognized result in case of event_id = EVENT_RECOGNIZED_UTT
# Also it contains a best-guess in case of event_id = EVENT_FAILED_DECODING
asrresult recognized_utterance


# The current state of the recognizer (activated/deactivated)
bool active
