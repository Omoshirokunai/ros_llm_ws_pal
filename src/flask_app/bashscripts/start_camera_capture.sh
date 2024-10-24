#!/bin/bash

# Run the camera capture script in the background
SCRIPT_PATH="/path/to/remote_camera_capture.py"

# Check if the script is already running
if pgrep -f "camera_capture.py" > /dev/null
then
    echo "camera_capture.py is already running."
else
    echo "Starting camera_capture.py..."
    nohup python2 $SCRIPT_PATH &  # Run in the background
    echo "camera_capture.py started."
fi
