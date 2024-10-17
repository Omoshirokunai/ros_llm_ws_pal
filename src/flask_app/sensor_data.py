
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
        command = "python /home/pal/catkin_ws/src/cam/remote_camera_reader.py"
        stdout, stderr = self.ssh_client.execute_command(command)
        # Retry mechanism to wait for the output
        # Read the image data from the text file
        command = "cat /home/pal/catkin_ws/src/cam/latest_image.txt"
        stdout, stderr = self.ssh_client.execute_command(command)

        if stderr:
            print(f"Error reading image data: {stderr}")
            return None
        # if stderr:
        #     print(f"Error: {stderr}")
        #     return None
        # if len(stdout) == 0:
        #     print("No data received from SSH")
        #     return None


        # image_data_base64 = stdout.strip()
        # # Debug: Check if any data is received
        # print(f"Received base64 data (first 100 characters): {image_data_base64[:100]}")

        # if not image_data_base64:
        #     print("No image data received")
        #     return None

        # # Decode the base64 data into image bytes in Python 3
        # image_data_bytes = base64.b64decode(image_data_base64)

        # # Convert the binary data to an image using PIL
        # image = Image.open(BytesIO(image_data_bytes))
        # image_array = np.array(image)

        # return image_array
        return stdout

if __name__ == "__main__":
    sensors = RobotSensors()
    camera_image = sensors.get_camera_data()
    if camera_image is not None:
        print("Successfully retrieved camera image")
    else:
        print("Failed to retrieve camera image")
