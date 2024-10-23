
import base64
import re  # For cleaning up base64 strings
from io import BytesIO

import numpy as np
from PIL import Image
from ssh_with_ros import PersistentSSHClient, SingleCommandSSHClient


class RobotSensors:
    def __init__(self):
        self.ssh_client = SingleCommandSSHClient()

    def get_camera_data(self):


if __name__ == "__main__":
    sensors = RobotSensors()
    camera_image = sensors.get_camera_data()
    if camera_image is not None:
        print("Successfully retrieved camera image")
    else:
        print("Failed to retrieve camera image")

#--- new
import os
import paramiko
from flask import Flask, send_file

app = Flask(__name__)

# SSH connection details
SSH_USER = 'your_ssh_user'
SSH_HOST = 'your_ssh_host'
SSH_PORT = 22  # Default SSH port
REMOTE_IMAGE_PATH = '/path/to/save/images/current.jpg'  # Update to the correct path
LOCAL_IMAGE_PATH = '/path/to/save/images/current.jpg'   # Path where you want to store the local copy

# Function to fetch the image via SSH
def fetch_image_via_ssh():
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    ssh_client.connect(SSH_HOST, username=SSH_USER, port=SSH_PORT, password='your_password')

    sftp = ssh_client.open_sftp()
    
    try:
        # Fetch the image from the remote server
        sftp.get(REMOTE_IMAGE_PATH, LOCAL_IMAGE_PATH)
    except Exception as e:
        print(f"Failed to fetch image: {e}")
    finally:
        sftp.close()
        ssh_client.close()

# Route to serve the image
@app.route('/video_feed')
def video_feed():
    # Fetch the latest image from the remote machine
    fetch_image_via_ssh()

    # Serve the image file
    return send_file(LOCAL_IMAGE_PATH, mimetype='image/jpeg')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

