# Request messages for the recognizer service.
# It is possible to request and activate task, 
# a grammar management task and language change or just
# request the current status.

# Type of request list
int8 ACTIVATION = 1
int8 GRAMMAR = 2
int8 LANGUAGE = 3
int8 STATUS = 4
int8 KWSPOTTING = 5

# Message variables
# list of requests types (several requests can be send in one single message)
int8[] requests

# Information related to each possible request
# except for status that does not need any additional info.
ASRActivation activation
ASRLangModelMngmt model
ASRLanguage lang
