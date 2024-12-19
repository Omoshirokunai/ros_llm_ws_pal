#!/bin/bash

# Run the camera capture script in the background
SCRIPT_PATH="/home/pal/catkin_ws/src/get_tiago_camera/src/remote_camera_reader.py"

# Check if the script is already running
if pgrep -f "remote_camera_reader.py" > /dev/null
then
    echo "camera_capture.py is already running."
else
    echo "Bash Starting camera_capture.py..."
    # nohup python2 $SCRIPT_PATH &  # Run in the background
    python2 $SCRIPT_PATH &
    echo "camera_capture.py started."
fi
