# lidar_safety.py
import json
import os
import time

import numpy as np


class LidarSafety:
    def __init__(self):

        self.SAFE_DISTANCE = 0.01  # meters
        self.WARNING_DISTANCE = 0.001  # meters
        self.CRITICAL_DISTANCE = 0.001  # meters

        self.safety_threshold = 0.0021  # 50cm minimum safe distance
        self.field_of_view = 180  # degrees
        self.lidar_path = os.path.join('src/flask_app/static/images/lidar_data.json')
        # Add metric tracking
        self.collision_checks = 0
        self.safety_history = {
            "last_violations": [],
            "recovery_actions": [],

        }
        self.min_distances = []

        # Sector definitions (in degrees)
        self.sectors = {
            "forward": (80, 100),    # 20° forward cone
            "left": (150, 170),      # 20° left cone
            "right": (10, 30),       # 20° right cone
            "periphery": (30, 150)    # Wider awareness zone
        }

        self.safety_metrics = {
            "violations": {"forward": 0, "left": 0, "right": 0},
            "close_calls": {"forward": 0, "left": 0, "right": 0},
            "min_distances": []
        }

        # Sector definitions (in degrees)
        self.sectors = {
            "forward": (80, 100),    # 20° forward cone
            "left": (150, 170),      # 20° left cone
            "right": (10, 30),       # 20° right cone
            "periphery": (30, 150)    # Wider awareness zone
        }

        self.safety_metrics = {
            "violations": {"forward": 0, "left": 0, "right": 0},
            "close_calls": {"forward": 0, "left": 0, "right": 0},
            "min_distances": []
        }

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
        ranges = self.load_lidar_data()
        if not ranges:
            return False, "LIDAR data unavailable"

        ranges = np.array(ranges)
        timestamp = time.time()
        # Get relevant sector for action
        if "forward" in action:
            sector = self.sectors["forward"]
            direction = "forward"
        elif "left" in action:
            sector = self.sectors["left"]
            direction = "left"
        elif "right" in action:
            sector = self.sectors["right"]
            direction = "right"
        else:
            return False, "Invalid action"

        # Calculate safety metrics
        sector_ranges = ranges[sector[0]:sector[1]]
        # min_distance = np.min(sector_ranges)
        # mean_distance = np.mean(sector_ranges)
        # Calculate metrics with nan handling
        min_distance = np.nan_to_num(np.min(sector_ranges), nan=float('inf'))
        mean_distance = np.nan_to_num(np.mean(sector_ranges), nan=float('inf'))


        # Log metrics
        self.safety_metrics["min_distances"].append({
            "timestamp": timestamp,
            "direction": direction,
            "min_distance": min_distance,
            "mean_distance": mean_distance
        })

        # Safety checks
        if min_distance < self.CRITICAL_DISTANCE:
            self.safety_metrics["violations"][direction] += 1
            return False, f"CRITICAL: Obstacle at {min_distance:.4f}m in {direction} direction"

        elif min_distance < self.WARNING_DISTANCE:
            self.safety_metrics["close_calls"][direction] += 1
            return False, f"WARNING: Limited clearance of {min_distance:.4f}m in {direction} direction"

         # Check peripheral safety for forward motion
        if "forward" in action:
            peripheral_ranges = ranges[self.sectors["periphery"][0]:self.sectors["periphery"][1]]
            if np.min(peripheral_ranges) < self.WARNING_DISTANCE:
                return False, f"CAUTION: Tight passage, {np.min(peripheral_ranges):.4f}m clearance"

        return True, f"Safe to proceed, {min_distance:.4f}m clearance"

    def get_safety_context(self):
        """Return safety context for decision making"""
        return {
            "recent_violations": self.safety_history["last_violations"][-5:],
            "high_risk_directions": [
                dir for dir, count in self.safety_metrics["violations"].items()
                if count > 0
            ]
        }
