import os

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()
output_dir = "/path/to/save/images"  # Update to the directory where you want to save images
image_received = False
def save_image_as_jpeg(image_data):
    try:
        # Convert ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(image_data, desired_encoding="bgr8")

        # Define paths for current and previous images
        current_image_path = os.path.join(output_dir, "current.jpg")
        previous_image_path = os.path.join(output_dir, "previous.jpg")

        # If current.jpg exists, move it to previous.jpg
        if os.path.exists(current_image_path):
            os.rename(current_image_path, previous_image_path)

        # Save the new image as current.jpg
        cv2.imwrite(current_image_path, cv_image)
image_received = True
        print(f"Image saved as {current_image_path}")
    except Exception as e:
        print(f"Failed to save image: {e}")

def image_callback(msg):
    global image_received
    if not image_received:
        save_image_as_jpeg(msg)
        # Shutdown ROS node after the first image is captured
        rospy.signal_shutdown('Image captured, shutting down node.')

def main():
    rospy.init_node('image_saver_node')

    # Subscribe to the camera topic
    rospy.Subscriber('/xtion/rgb/image_raw/compressed', Image, image_callback)

    print("Waiting for images...")
    rospy.spin()

if __name__ == '__main__':
    main()
