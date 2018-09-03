# This message is to be used in the ASR service to manage the grammars
# makes possible to enable/disable, load/unload grammars.


# Types of action
int8 ENABLE = 1
int8 DISABLE = 2
int8 LOAD = 3
int8 UNLOAD = 4

# Message variables
# Type of action requested
int8 action

# Name of the grammar to actuate on.
string modelName
