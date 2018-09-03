#This represents a 2-D motion model map, in which each cell has the probabilistic motion models for the robot.

Header header 

#MetaData for the map
nav_msgs/MapMetaData info

# The map data, in row-major order, starting with (0,0). A list of  models is stored for every cell. 
MotionModelList[] data

