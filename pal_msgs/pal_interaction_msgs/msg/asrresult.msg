## Message that containes the recognized utterance.
## Confidence values
int8 CONFIDENCE_UNDEFINED = -1
int8 CONFIDENCE_POOR = 1
int8 CONFIDENCE_LOW  = 2
int8 CONFIDENCE_GOOD = 3
int8 CONFIDENCE_MAX  = 4

# ASR result messages used by RosRecognizerServer

# text recognized
string text

# confidence with values from POOR to MAX
int8 confidence

# start and end of the recognizer uterance.
time start
time end

# list of recognized tags
# key value pairs of strings extracted from the text
# given the action tags placed in the grammar.
actiontag[] tags
