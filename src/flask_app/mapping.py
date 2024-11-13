from threading import Lock

import cv2
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


class SimpleMapper:
    def __init__(self):
        # Map parameters
        self.resolution = 0.05  # meters per pixel
        self.width = 400  # pixels
        self.height = 400  # pixels
        self.map_center = (self.width // 2, self.height // 2)

        # Initialize map array
        self.map = np.zeros((self.height, self.width), dtype=np.uint8)
        self.map_lock = Lock()

        # Robot pose
        self.x = 0
        self.y = 0
        self.theta = 0

        # ROS setup
        rospy.init_node('simple_mapper', anonymous=True)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
        rospy.Subscriber('/scan_raw', LaserScan, self.lidar_callback)
        rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        # Update robot pose
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([orientation.x, orientation.y,
                                                orientation.z, orientation.w])

    def world_to_map(self, wx, wy):
        # Convert world coordinates to map pixels
        mx = int(self.map_center[0] + wx / self.resolution)
        my = int(self.map_center[1] + wy / self.resolution)
        return mx, my

    def lidar_callback(self, scan):
        with self.map_lock:
            # Get robot position in map coordinates
            rx, ry = self.world_to_map(self.x, self.y)

            # Process each laser reading
            angle = scan.angle_min
            for r in scan.ranges:
                if scan.range_min <= r <= scan.range_max:
                    # Convert laser reading to world coordinates
                    wx = self.x + r * np.cos(angle + self.theta)
                    wy = self.y + r * np.sin(angle + self.theta)

                    # Convert to map coordinates
                    mx, my = self.world_to_map(wx, wy)

                    # Mark occupied cells
                    if 0 <= mx < self.width and 0 <= my < self.height:
                        self.map[my, mx] = 255  # Mark as occupied

                        # Draw line from robot to obstacle
                        cv2.line(self.map, (rx, ry), (mx, my), 128, 1)

                angle += scan.angle_increment

            # Save map periodically
            self.save_map()

    def save_map(self):
        cv2.imwrite('current_map.jpg', self.map)

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            # Publish map for visualization
            self.publish_map()
            rate.sleep()

    def publish_map(self):
        # Convert map to ROS OccupancyGrid message
        grid = OccupancyGrid()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = 'map'
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.data = (self.map.flatten() * 100 / 255).astype(int).tolist()
        self.map_pub.publish(grid)

if __name__ == '__main__':
    try:
        mapper = SimpleMapper()
        mapper.run()
    except rospy.ROSInterruptException:
        pass