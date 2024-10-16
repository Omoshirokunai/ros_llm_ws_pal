
import base64

import numpy as np
from PIL import Image
from ssh_with_ros import PersistentSSHClient, SingleCommandSSHClient


class RobotSensors:
    def __init__(self):
        self.ssh_client = PersistentSSHClient()

    def get_camera_data(self):
        command = "rostopic echo /xtion/rgb/image_raw/compressed -n 1"
        stdout, stderr = self.ssh_client.execute_command(command)
        if stderr:
            print(f"Error: {stderr}")
            return None
         # Extract the base64 encoded image data from the stdout
        data_line = next((line for line in stdout.split('\n') if line.startswith('data:')), None)
        if data_line:
            image_data = data_line.split('data: ')[1]
            # print(image_data)

            print(f"Length of camera data: {len(image_data)}")
            missing_padding = len(image_data) % 4
            if missing_padding:
                image_data += '=' * (4 - missing_padding)

            image_data_bytes = base64.b64decode(image_data)
            # image_array = np.array(image_data)
            return image_data_bytes
            # return image_data
        # return np.zeros((480, 640, 3), dtype=np.uint8)
        return None


    def get_lidar_data(self):
        pass

