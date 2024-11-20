
import os
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

        #map directory
        self.map_dir = os.path.join(os.path.dirname(__file__), 'maps')
        os.makedirs(self.map_dir, exist_ok=True)
        self.map_path = os.path.join(self.map_dir, 'map.jpg')
        self.prev_map_path = os.path.join(self.map_dir, 'prev_map.jpg')

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
            # Save previous map
            if os.path.exists(self.map_path):
                os.rename(self.map_path, self.prev_map_path)

            vis_map = cv2.cvtColor(self.map, cv2.COLOR_GRAY2BGR)

            # Draw robot position at center
            # cv2.circle(vis_map, self.map_center, 5, (0, 0, 255), -1)
             # Draw robot position with heading
            heading = self.get_robot_heading()
            cv2.arrowedLine(vis_map,
                          self.map_center,
                          (int(self.map_center[0] + 20*np.cos(heading)),
                           int(self.map_center[1] + 20*np.sin(heading))),
                           (0, 0, 255), 2)

            # Draw grid lines
            grid_spacing = int(1.0 / self.resolution)
            for i in range(0, self.width, grid_spacing):
                cv2.line(vis_map, (i, 0), (i, self.height-1), (50, 50, 50), 1)
                cv2.line(vis_map, (0, i), (self.width-1, i), (50, 50, 50), 1)

            # grid_spacing = int(1.0 / self.resolution)
            # for i in range(0, self.width, grid_spacing):
            #     cv2.line(vis_map, (i, 0), (i, self.height-1), (50, 50, 50), 1)
            # for i in range(0, self.height, grid_spacing):
            #     cv2.line(vis_map, (0, i), (self.width-1, i), (50, 50, 50), 1)

            # cv2.imwrite('map.jpg', vis_map)
            cv2.imwrite(self.map_path, vis_map)

if __name__ == '__main__':
    try:
        mapper = OccupancyMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

