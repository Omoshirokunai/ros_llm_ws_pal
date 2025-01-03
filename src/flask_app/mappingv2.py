# mapping_service.py
import json
import logging
import os
import time
from threading import Lock

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
        self.width = 500
        self.height = 500
        self.resolution = 0.05  # meters per pixel
        self.map_center = (self.width // 2, self.height // 2)
        self.map = np.zeros((self.height, self.width), dtype=np.uint8)
        self.persistence_map = np.zeros((self.height, self.width), dtype=np.uint8)
        # self.save_path = os.path.expanduser('~/map.jpg')
        # self.save_path = '/home/pal/catkin_ws/src/get_map_image/map.jpg'
        # self.json_path = '/home/pal/catkin_ws/src/get_map_image/lidar_data.json'
        self.save_path = 'src/flask_app/static/images/sim_map.jpg'
        self.json_path = 'src/flask_app/static/images/lidar_data.json'

        self.latest_scan = None

        #locks of map and latest_scan
        self.map_lock = Lock()
        self.lidar_lock = Lock()

        # Decay and threshold parameters
        self.decay_factor = 0.7  # Increased decay
        self.persistence_threshold = 50  # Minimum value to keep point
        self.update_counter = 0
        self.clear_interval = 10  # Clear map every N updates

def laser_callback(scan, map_data):
    """Process LIDAR scan data and update maps"""
    try:
        # Clear current map
        map_data.map = np.zeros_like(map_data.map)

        # Store raw LIDAR data
        map_data.latest_scan = {
            'ranges': list(scan.ranges),
            'angle_min': scan.angle_min,
            'angle_max': scan.angle_max,
            'angle_increment': scan.angle_increment,
            'timestamp': time.time()
        }

        # Save LIDAR data to JSON
        # with open(map_data.lidar_path, 'w') as f:
        #     json.dump(map_data.latest_scan, f)

        # Atomic write of LIDAR data
        temp_json = f"{map_data.json_path}.temp"
        with open(temp_json, 'w') as f:
            json.dump(map_data.latest_scan, f)
        os.rename(temp_json, map_data.json_path)

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

         # Periodic clear of persistence map
        map_data.update_counter += 1
        if map_data.update_counter >= map_data.clear_interval:
            map_data.persistence_map *= 0.5  # Aggressive clear
            map_data.update_counter = 0

        # Update persistence map with decay
        map_data.persistence_map = np.maximum(
            map_data.persistence_map * map_data.decay_factor, # Decay factor
            map_data.map
        )
        # Apply threshold to remove weak persistent points
        map_data.persistence_map[map_data.persistence_map < map_data.persistence_threshold] = 0


        # Generate visualization
        vis_map = cv2.cvtColor(map_data.persistence_map.astype(np.uint8), cv2.COLOR_GRAY2BGR)

        # Draw robot position
        cv2.circle(vis_map, map_data.map_center, 5, (0, 0, 255), -1)

        # Draw obstacles with larger circles
        obstacle_points = np.where(map_data.persistence_map > map_data.persistence_threshold)
        for y, x in zip(obstacle_points[0], obstacle_points[1]):
            cv2.circle(vis_map, (x, y), 2, (255, 255, 255), -1)  # Increased radius from 1 to 3

        # Draw direction arrow
        # Calculate arrow endpoint (30 pixels in front of robot)
        arrow_length = 10
        robot_x, robot_y = map_data.map_center
        # Assuming robot's orientation is in radians
        robot_angle = 0  # You'll need to get this from your robot's odometry
        end_x = int(robot_x + arrow_length * np.cos(robot_angle))
        end_y = int(robot_y + arrow_length * np.sin(robot_angle))

        # Draw arrow
        cv2.arrowedLine(vis_map,
                        map_data.map_center,
                        (end_x, end_y),
                        (0, 255, 0),  # Green color
                        2,  # Thickness
                        tipLength=0.3)  # Arrow head size
        # # Draw grid
        # grid_spacing = int(1.0 / map_data.resolution)
        # for i in range(0, map_data.width, grid_spacing):
        #     cv2.line(vis_map, (i, 0), (i, map_data.height-1), (50, 50, 50), 1)
        # for i in range(0, map_data.height, grid_spacing):
        #     cv2.line(vis_map, (0, i), (map_data.width-1, i), (50, 50, 50), 1)

        # Save map
        # cv2.imwrite(map_data.save_path, vis_map)
        temp_map = f"{map_data.save_path}.temp"
        cv2.imwrite(temp_map, vis_map)
        os.rename(temp_map, map_data.save_path)

    except Exception as e:
        logging.error(e)

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
        rate = rospy.Rate(5)  # 2 Hz update rate
        while not rospy.is_shutdown():
            rate.sleep()

    except Exception as e:
        logging.error(e)
    finally:
        logging.info("Mapping service stopped")

if __name__ == '__main__':
    main()

# # mapping_service.py
# import json
# import logging
# import os
# import time
# from threading import Lock

# import cv2
# import numpy as np
# import rospy
# from nav_msgs.msg import Odometry  # Add for robot pose
# from sensor_msgs.msg import LaserScan

# # Configure logging to show on console too
# logging.basicConfig(
#     level=logging.INFO,
#     format='%(asctime)s - %(levelname)s - %(message)s',
#     handlers=[
#         logging.FileHandler('mapping_service.log'),
#         logging.StreamHandler()
#     ]
# )

# class MapData:
#     def __init__(self):
#         self.width = 500
#         self.height = 500
#         self.resolution = 0.05  # meters per pixel
#         self.map_center = (self.width // 2, self.height // 2)
#         self.map = np.zeros((self.height, self.width), dtype=np.uint8)
#         self.persistence_map = np.zeros((self.height, self.width), dtype=np.uint8)

#         # Update paths for simulation
#         self.save_path = 'static/images/map.jpg'
#         self.json_path = 'static/images/lidar_data.json'

#         # Create directories if they don't exist
#         os.makedirs('static/images', exist_ok=True)

#         self.latest_scan = None
#         self.robot_pose = None  # Store robot pose

#         self.map_lock = Lock()
#         self.lidar_lock = Lock()

#         # Decay and threshold parameters
#         self.decay_factor = 0.7
#         self.persistence_threshold = 50
#         self.update_counter = 0
#         self.clear_interval = 10

# def odom_callback(odom_msg, map_data):
#     """Store robot pose from odometry"""
#     with map_data.map_lock:
#         map_data.robot_pose = odom_msg.pose.pose

# def laser_callback(scan, map_data):
#     """Process LIDAR scan data and update maps"""
#     try:
#         logging.info("Received LIDAR scan")

#         with map_data.map_lock:
#             # Clear current map
#             map_data.map = np.zeros_like(map_data.map)

#             # Store raw LIDAR data
#             map_data.latest_scan = {
#                 'ranges': list(scan.ranges),
#                 'angle_min': scan.angle_min,
#                 'angle_max': scan.angle_max,
#                 'angle_increment': scan.angle_increment,
#                 'timestamp': time.time()
#             }

#             # Save LIDAR data atomically
#             temp_json = f"{map_data.json_path}.temp"
#             with open(temp_json, 'w') as f:
#                 json.dump(map_data.latest_scan, f)
#             os.rename(temp_json, map_data.json_path)

#             # Process LIDAR data
#             angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment,
#                              scan.angle_increment)
#             ranges = np.array(scan.ranges)

#             # Filter valid readings
#             valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
#             angles = angles[valid]
#             ranges = ranges[valid]

#             if len(ranges) == 0:
#                 logging.warning("No valid LIDAR readings")
#                 return

#             # Rest of processing remains the same...
#             # Save visualization
#              # Generate visualization
#             vis_map = cv2.cvtColor(map_data.persistence_map.astype(np.uint8), cv2.COLOR_GRAY2BGR)

#             temp_map = f"{map_data.save_path}.temp"
#             cv2.imwrite(temp_map, vis_map)
#             os.rename(temp_map, map_data.save_path)

#             logging.info(f"Map updated with {len(ranges)} points")

#     except Exception as e:
#         logging.error(f"Error in laser_callback: {e}")

# def main():
#     try:
#         # Initialize ROS node
#         rospy.init_node('mapping_service', anonymous=True)
#         logging.info("Mapping service started")

#         # Initialize map data
#         map_data = MapData()

#         # Subscribe to correct topics for simulation
#         rospy.Subscriber(
#             '/scan',  # Changed from /scan_raw to /scan for simulation
#             LaserScan,
#             callback=lambda scan: laser_callback(scan, map_data),
#             queue_size=1
#         )

#         # Add odometry subscription
#         rospy.Subscriber(
#             '/mobile_base_controller/odom',
#             Odometry,
#             callback=lambda odom: odom_callback(odom, map_data),
#             queue_size=1
#         )

#         logging.info("Waiting for LIDAR data...")

#         # Keep running
#         rate = rospy.Rate(5)
#         while not rospy.is_shutdown():
#             if map_data.latest_scan:
#                 logging.info("Mapping service running - receiving data")
#             rate.sleep()

#     except Exception as e:
#         logging.error(f"Error in main: {e}")
#     finally:
#         logging.info("Mapping service stopped")

# if __name__ == '__main__':
#     main()