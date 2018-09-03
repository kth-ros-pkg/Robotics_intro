#!/bin/bash

if [[ $# < 1 ]]
then
    echo "Usage: $0 <input bag>"
    echo "Usage: $0 <input bag> <output bag>"
    exit 1
fi

if [[ $# < 2 ]]
then
    path=$1
    OUTPUT=${path//.bag/_filtered.bag}
else
    OUTPUT=$2
fi

INPUT=$1

rosbag filter $INPUT $OUTPUT "topic != '/tf' or m.transforms[0].header.frame_id != 'map' and m.transforms[0].child_frame_id != 'odom'"
