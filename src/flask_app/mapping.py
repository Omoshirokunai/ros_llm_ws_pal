# mapping.py

# from threading import Lock

# import cv2
# import numpy as np
# import rospy
# from cv_bridge import CvBridge
# from nav_msgs.msg import OccupancyGrid
# from sensor_msgs.msg import LaserScan
# src/flask_app/mapping.py

import os
import threading

import cv2
import numpy as np
import rospy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from PIL import Image
from sensor_msgs.msg import LaserScan

# class SLAMMapper:
#     def __init__(self):
#         # Initialize ROS node
#         rospy.init_node('slam_mapper', anonymous=True)

#         # Initialize variables
#         self.map_data = None
#         self.map_lock = Lock()
#         self.bridge = CvBridge()
#         self.map_resolution = 0.1  # meters per pixel
#         self.map_width = 400      # pixels
#         self.map_height = 4000     # pixels

#         # Subscribers
#         # rospy.Subscriber('/scan_raw', LaserScan, self.lidar_callback)
#         rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

#         # Timer for saving map
#         rospy.Timer(rospy.Duration(3.0), self.save_map)

#     def lidar_callback(self, scan):
#         # Process incoming lidar data
#         with self.map_lock:
#             if self.map_data is None:
#                 self.map_data = np.zeros((self.map_height, self.map_width))

#     def map_callback(self, map_msg):
#         """Handle incoming map data from gmapping"""
#         with self.map_lock:
#             # Convert occupancy grid to numpy array
#             width = map_msg.info.width
#             height = map_msg.info.height

#             if width == 0 or height == 0:
#                 rospy.logwarn("Received empty map")
#                 return

#             # Convert from 1D array to 2D
#             grid_data = np.array(map_msg.data).reshape((height, width))
#             image = np.ones((height, width), dtype=np.uint8) * 128
#             image[grid_data == 0] = 255    # Free space
#             image[grid_data == 100] = 0    # Obstacles

#             # Resize to desired dimensions
#             self.map_data = cv2.resize(image, (self.map_width, self.map_height),
#                                      interpolation=cv2.INTER_NEAREST)
#     def save_map(self, event=None):
#         with self.map_lock:
#             if self.map_data is not None:
#                 cv2.imwrite('current_map.jpg', self.map_data)
#                 rospy.loginfo("Map saved to current_map.jpg")

#     # def run(self):
#     #     rospy.spin()


class SLAMMapper:
    def __init__(self):
        rospy.init_node('slam_mapper', anonymous=True)

        # Initialize locks
        self.map_lock = threading.Lock()
        self.scan_lock = threading.Lock()

        # Initialize map data
        self.map_data = None
        self.map_info = None

        # Subscribe to topics
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        # Create map directory if it doesn't exist
        self.map_dir = os.path.join(os.path.dirname(__file__), 'maps')
        os.makedirs(self.map_dir, exist_ok=True)

    def lidar_callback(self, scan):
        with self.scan_lock:
            # Process LIDAR scan data
            ranges = np.array(scan.ranges)
            angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

            # Filter invalid readings
            valid_mask = np.isfinite(ranges)
            ranges = ranges[valid_mask]
            angles = angles[valid_mask]

            # Convert to cartesian coordinates
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)

            # Store processed scan data
            self.latest_scan = np.vstack((x, y))

    def map_callback(self, map_msg):
        """Convert occupancy grid to image format"""
        with self.map_lock:
            # Convert map data to numpy array
            map_array = np.array(map_msg.data)
            map_array = map_array.reshape((map_msg.info.height, map_msg.info.width))

            # Convert to image format (0-255)
            map_image = np.zeros_like(map_array, dtype=np.uint8)
            map_image[map_array == 0] = 255    # Free space (white)
            map_image[map_array == 100] = 0    # Obstacles (black)
            map_image[map_array == -1] = 128   # Unknown (gray)

            # Store map data and metadata
            self.map_data = map_image
            self.map_info = map_msg.info

            # Save the current map
            self.save_map()

    def save_map(self, event=None):
        """Save the current map as an image"""
        if self.map_data is not None:
            map_path = os.path.join(self.map_dir, 'current_map.png')
            cv2.imwrite(map_path, self.map_data)

            # Also save a version with LIDAR overlay if available
            if hasattr(self, 'latest_scan'):
                overlay_map = self.create_overlay_map()
                overlay_path = os.path.join(self.map_dir, 'map_with_scan.png')
                cv2.imwrite(overlay_path, overlay_map)

    def create_overlay_map(self):
        """Create a map with current LIDAR scan overlay"""
        if self.map_data is None or not hasattr(self, 'latest_scan'):
            return None

        overlay = self.map_data.copy()
        # Convert scan points to map coordinates
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        x_map = ((self.latest_scan[0] - origin_x) / resolution).astype(int)
        y_map = ((self.latest_scan[1] - origin_y) / resolution).astype(int)

        # Plot scan points
        valid_points = (x_map >= 0) & (x_map < overlay.shape[1]) & \
                      (y_map >= 0) & (y_map < overlay.shape[0])
        overlay[y_map[valid_points], x_map[valid_points]] = 100  # Mark scan points

        return overlay

    def get_current_map(self):
        """Return the current map image"""
        with self.map_lock:
            return self.map_data.copy() if self.map_data is not None else None

    def run(self):
        """Main run loop"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.save_map()
            rate.sleep()



if __name__ == '__main__':
    try:
        mapper = SLAMMapper()
        rospy.spin()
        # mapper.run()
    except rospy.ROSInterruptException:
        pass

