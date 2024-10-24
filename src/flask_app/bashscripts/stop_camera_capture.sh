#!/bin/bash

# Find the process ID (PID) of camera_capture.py
PID=$(pgrep -f "remote_camera_capture.py")

# Check if the process is running
if [ -z "$PID" ]; then
    echo "camera_capture.py is not running."
else
    # Kill the process
    echo "Stopping camera_capture.py (PID: $PID)..."
    kill -9 $PID
    echo "camera_capture.py stopped."
fi
