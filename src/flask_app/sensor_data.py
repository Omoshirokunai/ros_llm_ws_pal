
import base64
from io import BytesIO
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
        # Extract base64 encoded image data from the stdout
        data_line = next((line for line in stdout.split('\n') if 'data:' in line), None)

        if data_line:
            # Extract the base64 image data after 'data: '
            image_data_base64 = data_line.split('data: ')[1].strip()

            # Fix missing padding in base64 string if necessary
            missing_padding = len(image_data_base64) % 4
            if missing_padding:
                image_data_base64 += '=' * (4 - missing_padding)

            # Decode base64 to get binary image data
            try:
                image_data_bytes = base64.b64decode(image_data_base64)

                # Convert the binary data to an image using PIL
                image = Image.open(BytesIO(image_data_bytes))

                # Convert image to a numpy array (useful for video streaming)
                image_array = np.array(image)

                return image_array  # Return as numpy array to stream via Flask

            except Exception as e:
                print(f"Failed to decode image data: {e}")
                return None
        return None

    # def get_camera_data(self):
    #     # ROS command to get one frame from the uncompressed RGB camera topic
    #     command = "rostopic echo /xtion/rgb/image_raw -n 1 --noarr"
    #     stdout, stderr = self.ssh_client.execute_command(command)

    #     if stderr:
    #         print(f"Error: {stderr}")
    #         return None

    #     # Find the binary PNG data in stdout
    #     image_data_lines = []
    #     collecting = False

    #     for line in stdout.splitlines():
    #         # ROS message will have binary data in base64-like format without `data:` tag
    #         if line.startswith('data:'):
    #             collecting = True
    #         if collecting:
    #             # ROS publishes hex-encoded binary data (in some cases in chunks)
    #             image_data_lines.append(line.strip())

    #     if image_data_lines:
    #         # Join the hex-encoded image data and convert it to raw binary
    #         image_data_hex = ''.join(image_data_lines)
    #         image_data_bytes = bytes.fromhex(image_data_hex)

    #         try:
    #             # Load the binary data into a PIL image
    #             image = Image.open(BytesIO(image_data_bytes))

    #             # Convert image to a numpy array (useful for video streaming)
    #             image_array = np.array(image)

    #             return image_array  # Return as numpy array to stream via Flask

    #         except Exception as e:
    #             print(f"Failed to decode image data: {e}")
    #             return None

    #     return None
         # Extract the base64 encoded image data from the stdout
        # data_line = next((line for line in stdout.split('\n') if line.startswith('data:')), None)
        # if data_line:
        #     image_data = data_line.split('data: ')[1]
        #     # print(image_data)

        #     print(f"Length of camera data: {len(image_data)}")
        #     missing_padding = len(image_data) % 4
        #     if missing_padding:
        #         image_data += '=' * (4 - missing_padding)

        #     image_data_bytes = base64.b64decode(image_data)
        #     # image_array = np.array(image_data)
        #     return image_data_bytes
        #     # return image_data
        # # return np.zeros((480, 640, 3), dtype=np.uint8)
        # return None


    def get_lidar_data(self):
        pass

