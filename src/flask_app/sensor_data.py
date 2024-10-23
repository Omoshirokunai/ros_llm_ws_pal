
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
