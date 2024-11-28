# mapping_service.py
import logging
import os
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    filename='mapping_service.log'
)

class MapData:
    def __init__(self):
        self.width = 800
        self.height = 800
        self.resolution = 0.05  # meters per pixel
        self.map_center = (self.width // 2, self.height // 2)
        self.map = np.zeros((self.height, self.width), dtype=np.uint8)
        self.persistence_map = np.zeros((self.height, self.width), dtype=np.uint8)
        self.save_path = os.path.expanduser('~/map.jpg')

def laser_callback(scan, map_data):
    """Process LIDAR scan data and update maps"""
    try:
        # Clear current map
        map_data.map = np.zeros_like(map_data.map)

        # Get LIDAR data
        angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment,
                         scan.angle_increment)
        ranges = np.array(scan.ranges)

        # Filter valid readings
        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
        angles = angles[valid]
        ranges = ranges[valid]

        # Convert to cartesian coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Convert to pixel coordinates
        px = (x / map_data.resolution + map_data.map_center[0]).astype(int)
        py = (y / map_data.resolution + map_data.map_center[1]).astype(int)

        # Filter valid points
        valid = (px >= 0) & (px < map_data.width) & (py >= 0) & (py < map_data.height)
        px = px[valid]
        py = py[valid]

        # Update current scan
        map_data.map[py, px] = 255

        # Update persistence map with decay
        map_data.persistence_map = np.maximum(
            map_data.persistence_map * 0.95,  # Decay factor
            map_data.map
        )

        # Generate visualization
        vis_map = cv2.cvtColor(map_data.persistence_map.astype(np.uint8), cv2.COLOR_GRAY2BGR)

        # Draw robot position
        cv2.circle(vis_map, map_data.map_center, 5, (0, 0, 255), -1)

        # Draw grid
        grid_spacing = int(1.0 / map_data.resolution)
        for i in range(0, map_data.width, grid_spacing):
            cv2.line(vis_map, (i, 0), (i, map_data.height-1), (50, 50, 50), 1)
        for i in range(0, map_data.height, grid_spacing):
            cv2.line(vis_map, (0, i), (map_data.width-1, i), (50, 50, 50), 1)

        # Save map
        cv2.imwrite(map_data.save_path, vis_map)

    except Exception as e:
        logging.error(f"Error in laser callback: {e}")

def main():
    try:
        # Initialize ROS node
        rospy.init_node('mapping_service', anonymous=True)
        logging.info("Mapping service started")

        # Initialize map data
        map_data = MapData()

        # Subscribe to LIDAR topic
        rospy.Subscriber(
            '/scan_raw',
            LaserScan,
            callback=lambda scan: laser_callback(scan, map_data),
            queue_size=1
        )

        # Keep running
        rate = rospy.Rate(2)  # 2 Hz update rate
        while not rospy.is_shutdown():
            rate.sleep()

    except Exception as e:
        logging.error(f"Error in main loop: {e}")
    finally:
        logging.info("Mapping service stopped")

if __name__ == '__main__':
    main()