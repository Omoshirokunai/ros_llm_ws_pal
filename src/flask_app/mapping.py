# mapping.py

from threading import Lock

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


class SLAMMapper:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('slam_mapper', anonymous=True)

        # Initialize variables
        self.map_data = None
        self.map_lock = Lock()
        self.bridge = CvBridge()
        self.map_resolution = 0.1  # meters per pixel
        self.map_width = 400      # pixels
        self.map_height = 4000     # pixels

        # Subscribers
        # rospy.Subscriber('/scan_raw', LaserScan, self.lidar_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Timer for saving map
        rospy.Timer(rospy.Duration(3.0), self.save_map)

    def lidar_callback(self, scan):
        # Process incoming lidar data
        with self.map_lock:
            if self.map_data is None:
                self.map_data = np.zeros((self.map_height, self.map_width))

    # def map_callback(self, map_msg):
    #     # Convert occupancy grid to image
    #     with self.map_lock:
    #         map_array = np.array(map_msg.data)
    #         map_array = map_array.reshape((map_msg.info.height, map_msg.info.width))
    #         # Convert to image format (0-255)
    #         map_image = np.zeros_like(map_array, dtype=np.uint8)
    #         map_image[map_array == 0] = 255    # Free space
    #         map_image[map_array == 100] = 0    # Obstacles
    #         map_image[map_array == -1] = 128   # Unknown
    #         self.map_data = map_image
    def map_callback(self, map_msg):
        """Handle incoming map data from gmapping"""
        with self.map_lock:
            # Convert occupancy grid to numpy array
            width = map_msg.info.width
            height = map_msg.info.height

            if width == 0 or height == 0:
                rospy.logwarn("Received empty map")
                return

            # Convert from 1D array to 2D
            grid_data = np.array(map_msg.data).reshape((height, width))

            # Convert to image format (0-255)
            # -1 (unknown) -> 128 (gray)
            # 0 (free) -> 255 (white)
            # 100 (occupied) -> 0 (black)
            image = np.ones((height, width), dtype=np.uint8) * 128
            image[grid_data == 0] = 255    # Free space
            image[grid_data == 100] = 0    # Obstacles

            # Resize to desired dimensions
            self.map_data = cv2.resize(image, (self.map_width, self.map_height),
                                     interpolation=cv2.INTER_NEAREST)
    def save_map(self, event=None):
        with self.map_lock:
            if self.map_data is not None:
                cv2.imwrite('current_map.jpg', self.map_data)
                rospy.loginfo("Map saved to current_map.jpg")

    # def run(self):
    #     rospy.spin()

if __name__ == '__main__':
    try:
        mapper = SLAMMapper()
        rospy.spin()
        # mapper.run()
    except rospy.ROSInterruptException:
        pass