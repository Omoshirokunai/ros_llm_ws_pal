# lidar_safety.py
import json
import os
import time

import numpy as np


class LidarSafety:
    def __init__(self):
        self.safety_threshold = 0.09  # 50cm minimum safe distance
        self.field_of_view = 180  # degrees
        self.lidar_path = os.path.join('src/flask_app/static/images/lidar_data.json')
        # Add metric tracking
        self.collision_checks = 0
        self.safety_violations = {
            "forward": 0,
            "left": 0,
            "right": 0
        }
        self.min_distances = []

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
        self.collision_checks += 1
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

         # Track minimum distances for each check
        for direction, sector in sectors.items():
            sector_ranges = ranges[sector[0]:sector[1]]
            min_dist = np.min(sector_ranges)
            self.min_distances.append({
                "timestamp": time.time(),
                "direction": direction,
                "distance": min_dist
            })

        if action == "move forward":
            sector = sectors["forward"]
            sector_ranges = ranges[sector[0]:sector[1]]
            min_distance = np.min(sector_ranges)

            if min_distance < self.safety_threshold:
                self.safety_violations["forward"] += 1
                return False, f"Obstacle detected {min_distance:.4f}m ahead"

        elif action == "turn left":
            sector = sectors["left"]
            sector_ranges = ranges[sector[0]:sector[1]]
            min_distance = np.min(sector_ranges)

            if min_distance < self.safety_threshold:
                self.safety_violations["left"] += 1
                return False, f"Obstacle detected {min_distance:.4f}m to the left"

        elif action == "turn right":
            sector = sectors["right"]
            sector_ranges = ranges[sector[0]:sector[1]]
            min_distance = np.min(sector_ranges)

            if min_distance < self.safety_threshold:
                self.safety_violations["right"] += 1
                return False, f"Obstacle detected {min_distance:.4f}m to the right"

        return True, None

    def get_safety_metrics(self):
        """Return aggregated safety metrics"""
        return {
            "total_checks": self.collision_checks,
            "total_violations": sum(self.safety_violations.values()),
            "violations_by_direction": self.safety_violations,
            "min_distance_history": self.min_distances
        }