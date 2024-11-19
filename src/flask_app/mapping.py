
import threading
import time

import cv2

# mapping.py
# mapping.py
import numpy as np

# mapping.py
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid
from PIL import Image
from sensor_msgs.msg import LaserScan

# class OccupancyMapper:
#     def __init__(self, width=800, height=800, resolution=0.05):
#         # Initialize map parameters
#         self.width = width  # pixels
#         self.height = height  # pixels
#         self.resolution = resolution  # meters per pixel
#         self.map_center = (width // 2, height // 2)

#         # Create empty map
#         self.map = np.zeros((height, width), dtype=np.uint8)
#         self.map_lock = threading.Lock()

#         # Initialize ROS node and subscriber
#         if not rospy.get_node_uri():
#             rospy.init_node('occupancy_mapper', anonymous=True)
#         self.laser_sub = rospy.Subscriber('/scan_raw', LaserScan, self.laser_callback)

#         # Start update thread
#         self.update_thread = threading.Thread(target=self.update_loop)
#         self.update_thread.daemon = True
#         self.update_thread.start()

#     def laser_callback(self, scan):
#         with self.map_lock:
#             # Clear the map for new update
#             self.map = np.zeros_like(self.map)

#             # Extract LIDAR data
#             angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment,
#                              scan.angle_increment)
#             ranges = np.array(scan.ranges)

#             # Filter invalid readings
#             valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
#             angles = angles[valid]
#             ranges = ranges[valid]

#             # Convert polar to cartesian coordinates
#             x = ranges * np.cos(angles)
#             y = ranges * np.sin(angles)

#             # Scale to pixels and shift to map center
#             px = (x / self.resolution + self.map_center[0]).astype(int)
#             py = (y / self.resolution + self.map_center[1]).astype(int)

#             # Filter points within map bounds
#             valid = (px >= 0) & (px < self.width) & (py >= 0) & (py < self.height)
#             px = px[valid]
#             py = py[valid]

#             # Mark occupied cells
#             self.map[py, px] = 255

#     def update_loop(self):
#         while not rospy.is_shutdown():
#             self.save_map()
#             time.sleep(2)  # Update every 2 seconds

#     def save_map(self):
#         with self.map_lock:
#             # Add visualization elements
#             vis_map = cv2.cvtColor(self.map, cv2.COLOR_GRAY2BGR)

#             # Draw robot position at center
#             cv2.circle(vis_map, self.map_center, 5, (0, 0, 255), -1)

#             # Draw grid lines
#             grid_spacing = int(1.0 / self.resolution)  # 1 meter grid
#             for i in range(0, self.width, grid_spacing):
#                 cv2.line(vis_map, (i, 0), (i, self.height-1), (50, 50, 50), 1)
#             for i in range(0, self.height, grid_spacing):
#                 cv2.line(vis_map, (0, i), (self.width-1, i), (50, 50, 50), 1)

#             # Save map
#             cv2.imwrite('map.jpg', vis_map)


class OccupancyMapper:
    def __init__(self):
        rospy.init_node('occupancy_mapper', anonymous=True)

        # Map parameters
        self.resolution = 0.05  # meters per pixel
        self.width = 1000  # pixels
        self.height = 1000  # pixels
        self.map_center = (self.width // 2, self.height // 2)

        # Initialize map
        self.map = np.zeros((self.height, self.width), dtype=np.uint8)
        self.map_lock = threading.Lock()

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribers
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # Start map update thread
        threading.Thread(target=self.update_loop, daemon=True).start()

    def laser_callback(self, msg):
        try:
            # Get robot pose
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y

            # Convert robot position to pixel coordinates
            robot_pixel_x = int(robot_x / self.resolution) + self.map_center[0]
            robot_pixel_y = int(robot_y / self.resolution) + self.map_center[1]

            with self.map_lock:
                # Draw current scan
                angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
                for angle, range_val in zip(angles, msg.ranges):
                    if range_val < msg.range_max:
                        # Convert polar to cartesian
                        x = range_val * np.cos(angle)
                        y = range_val * np.sin(angle)

                        # Convert to pixel coordinates
                        px = int(x / self.resolution) + robot_pixel_x
                        py = int(y / self.resolution) + robot_pixel_y

                        if 0 <= px < self.width and 0 <= py < self.height:
                            self.map[py, px] = 255

                            # Draw line from robot to point
                            cv2.line(self.map, (robot_pixel_x, robot_pixel_y),
                                   (px, py), 128, 1)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF Error: {e}")

    def update_loop(self):
        while not rospy.is_shutdown():
            self.save_map()
            time.sleep(2)

    def save_map(self):
        with self.map_lock:
            vis_map = cv2.cvtColor(self.map, cv2.COLOR_GRAY2BGR)

            # Draw robot position at center
            cv2.circle(vis_map, self.map_center, 5, (0, 0, 255), -1)

            # Draw grid lines
            grid_spacing = int(1.0 / self.resolution)
            for i in range(0, self.width, grid_spacing):
                cv2.line(vis_map, (i, 0), (i, self.height-1), (50, 50, 50), 1)
            for i in range(0, self.height, grid_spacing):
                cv2.line(vis_map, (0, i), (self.width-1, i), (50, 50, 50), 1)

            cv2.imwrite('map.jpg', vis_map)

if __name__ == '__main__':
    try:
        mapper = OccupancyMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

