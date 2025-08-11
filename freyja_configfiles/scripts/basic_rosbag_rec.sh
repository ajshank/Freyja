#! /bin/bash
# Records a rosbag with only essential topics. Uses a namespace
# provided as the first argument.
# Usage: >> basic_rosbag_rec.sh robot42
echo "Freyja: recording rosbag!"
ros2 bag record --no-discovery \
 ${1}/current_state \
 ${1}/reference_state \
 ${1}/controller_debug
 echo "Freyja: stopping rosbag process!"