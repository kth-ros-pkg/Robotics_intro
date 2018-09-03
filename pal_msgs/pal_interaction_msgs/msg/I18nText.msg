# section/key is used as in examples in the pal_tts_cfg pkg.
string section
string key

# language id, must be speficied using RFC 3066
string lang_id

# arguments contain the values by which 
# occurrences of '%s' will be replaced in the 
# main text.
# This only supports up to 2 arguments and no recursion.
# However, recursion and more argumnents are
# planned to be supported in the future.
I18nArgument[] arguments
