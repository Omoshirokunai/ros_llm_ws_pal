# lidar_safety.py
import json
import os

import numpy as np


class LidarSafety:
    def __init__(self):
        self.safety_threshold = 0.5  # 50cm minimum safe distance
        self.field_of_view = 180  # degrees
        self.lidar_path = os.path.join('static/images/lidar_data.json')

    def load_lidar_data(self):
        """Load latest LIDAR readings from file"""
        try:
            with open(self.lidar_path, 'r') as f:
                data = json.load(f)
                return data['ranges']  # Array of distances
        except Exception as e:
            print(f"Error loading LIDAR data: {e}")
            return None

    def check_direction_safety(self, action):
        """Check if action is safe based on LIDAR data"""
        ranges = self.load_lidar_data()
        if not ranges:
            return False, "LIDAR data unavailable"

        # Convert ranges to numpy array
        ranges = np.array(ranges)

        # Define direction sectors (assuming 180 degree FOV with readings every 1 degree)
        sectors = {
            "forward": (80, 100),  # Center 20 degrees
            "left": (150, 170),    # Leftmost 20 degrees
            "right": (10, 30),     # Rightmost 20 degrees
        }

        if action == "move forward":
            sector = sectors["forward"]
            sector_ranges = ranges[sector[0]:sector[1]]
            min_distance = np.min(sector_ranges)

            if min_distance < self.safety_threshold:
                return False, f"Obstacle detected {min_distance:.2f}m ahead"

        elif action == "turn left":
            sector = sectors["left"]
            sector_ranges = ranges[sector[0]:sector[1]]
            min_distance = np.min(sector_ranges)

            if min_distance < self.safety_threshold:
                return False, f"Obstacle detected {min_distance:.2f}m to the left"

        elif action == "turn right":
            sector = sectors["right"]
            sector_ranges = ranges[sector[0]:sector[1]]
            min_distance = np.min(sector_ranges)

            if min_distance < self.safety_threshold:
                return False, f"Obstacle detected {min_distance:.2f}m to the right"

        return True, None