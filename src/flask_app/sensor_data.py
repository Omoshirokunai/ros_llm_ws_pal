
import base64
from io import BytesIO
import re  # For cleaning up base64 strings
import numpy as np
from PIL import Image
from ssh_with_ros import PersistentSSHClient


class RobotSensors:
    def __init__(self):
        self.ssh_client = PersistentSSHClient()
    def get_camera_data(self):
        # Execute the Python 2 ROS node over SSH
        command = "python2 ros_camera_subscriber.py"
        stdout, stderr = self.ssh_client.execute_command(command)

        if stderr:
            print(f"Error: {stderr}")
            return None
        if len(stdout) == 0:
            print("No data received from SSH")

        image_data_base64 = stdout.strip()
         # Debug: Check if any data is received
        print(f"Received base64 data (first 100 characters): {image_data_base64[:100]}")

        if not image_data_base64:
            print("No image data received")
            return None

        # Decode the base64 data into image bytes in Python 3
        image_data_bytes = base64.b64decode(image_data_base64)

        # Convert the binary data to an image using PIL or OpenCV
        image = Image.open(BytesIO(image_data_bytes))
        image_array = np.array(image)

        return image_array
    # def get_camera_data(self):
    #     # Run the remote script that reads the image data
    #     command = "python3 remote_camera_reader.py"
    #     stdout, stderr = self.ssh_client.execute_command(command)

    #     if stderr:
    #         print(f"Error: {stderr}")
    #         return None

    #     # Get the base64-encoded image data from stdout
    #     image_data_base64 = stdout.strip()

    #     try:
    #         # Decode the base64-encoded image
    #         image_data_bytes = base64.b64decode(image_data_base64)

    #         # Convert the binary data to an image using PIL
    #         image = Image.open(BytesIO(image_data_bytes))

    #         # Convert image to a numpy array (useful for video streaming)
    #         image_array = np.array(image)

    #         return image_array  # Return as numpy array to stream via Flask

    #     except Exception as e:
    #         print(f"Failed to decode image data: {e}")
    #         return None

    # def get_camera_data(self):
    #     command = "rostopic echo /xtion/rgb/image_raw/compressed -n 1"
    #     stdout, stderr = self.ssh_client.execute_command(command)
    #     if stderr:
    #         print(f"Error: {stderr}")
    #         return None
    #     # Extract base64 encoded image data from the stdout
    #     data_line = next((line for line in stdout.split('\n') if 'data:' in line), None)


    #     if data_line:
    #         # Extract the base64 image data after 'data: '
    #         image_data_base64 = data_line.split('data: ')[1].strip()

    #         # Remove any non-base64 characters (newlines, spaces, etc.)
    #         image_data_base64 = re.sub(r'[^A-Za-z0-9+/=]', '', image_data_base64)

    #         # Fix missing padding in base64 string if necessary
    #         missing_padding = len(image_data_base64) % 4
    #         if missing_padding:
    #             image_data_base64 += '=' * (4 - missing_padding)

    #                     # Decode base64 to get binary image data
    #         try:
    #             image_data_bytes = base64.b64decode(image_data_base64)

    #             # Debug: Print the first few bytes of the decoded data
    #             print(f"Decoded image data (first 50 bytes): {image_data_bytes[:50]}")

    #             # Debug: Try writing the image data to a file to check if it's valid
    #             with open('debug_image_output.jpg', 'wb') as f:
    #                 f.write(image_data_bytes)

    #             # Convert the binary data to an image using PIL
    #             image = Image.open(BytesIO(image_data_bytes))

    #             # Convert image to a numpy array (useful for video streaming)
    #             image_array = np.array(image)

    #             return image_array  # Return as numpy array to stream via Flask


    #         except Exception as e:
    #             print(f"Failed to decode image data: {e}")
    #             return None

    #     return None

    def get_lidar_data(self):
        pass

