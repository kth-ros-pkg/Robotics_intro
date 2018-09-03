# This represents a 2-D grid map, in which each cell represents the signal strenght models of detected wifi networks.

Header header 

#MetaData for the map
nav_msgs/MapMetaData info

# Define the number of sectors to be used on wifi maps that include orientation info.
uint32 sectors

# The map data, in row-major order, starting with (0,0).  Wifi signal strenght models
# are gaussian probability distribution functions defined by mean and standard deviation value.
WifiSignalList[] data

