#!/bin/bash

# opens a ros2 bag file (as it's just a sqlite3 db) and dumps the first message to a hex file (as ascii)
# this is useful for debugging the contents of a ros2 bag file (primarily trying to interpret the cdr)
# see poc.ts for how this ascii file could be used

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <path_to_rosbag_db>"
    exit 1
fi

ROS_BAG_PATH=$1

sqlite3 "$ROS_BAG_PATH" <<EOF
.output output_file.hex
SELECT hex(data) FROM messages LIMIT 1;
EOF
