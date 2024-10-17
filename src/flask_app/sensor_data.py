
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
        command = "rostopic echo /xtion/rgb/image_raw/compressed -n 1"
        stdout, stderr = self.ssh_client.execute_command(command)
        if stderr:
            print(f"Error: {stderr}")
            return None
        # Extract base64 encoded image data from the stdout
        data_line = next((line for line in stdout.split('\n') if 'data:' in line), None)

        # if data_line:
        #     # Extract the base64 image data after 'data: '
        #     image_data_base64 = data_line.split('data: ')[1].strip()

        #     # Fix missing padding in base64 string if necessary
        #     missing_padding = len(image_data_base64) % 4
        #     if missing_padding:
        #         image_data_base64 += '=' * (4 - missing_padding)

        #     # Decode base64 to get binary image data
        #     try:
        #         image_data_bytes = base64.b64decode(image_data_base64)

        #         # Convert the binary data to an image using PIL
        #         image = Image.open(BytesIO(image_data_bytes))

        #         # Convert image to a numpy array (useful for video streaming)
        #         image_array = np.array(image)

        #         return image_array  # Return as numpy array to stream via Flask
        if data_line:
            # Extract the base64 image data after 'data: '
            image_data_base64 = data_line.split('data: ')[1].strip()

            # Remove any non-base64 characters (newlines, spaces, etc.)
            image_data_base64 = re.sub(r'[^A-Za-z0-9+/=]', '', image_data_base64)

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

                # Check if the image array has the correct format (3 channels for RGB)
                if len(image_array.shape) == 3:
                    return image_array  # Return as numpy array to stream via Flask
                else:
                    print(f"Unexpected image shape: {image_array.shape}")
                    return None

            except Exception as e:
                print(f"Failed to decode image data: {e}")
                return None

        return None

    def get_lidar_data(self):
        pass

