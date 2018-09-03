# Names of the actuators whose current limit is being set
string[] actuator_names

# Actuator current limits. Values belong to the [0, 1] interval, are
# non-dimensional and represent the fraction of an actuator's maximum allowed
# current, e.g., 0.5 would set an actuator's current limit to half its
# maximum value.
float64[] current_limits

