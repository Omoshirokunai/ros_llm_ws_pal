"""
    Get Images from Tiago Robot in simulation save them as current.jpg and previous.jpg
"""
import os

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()
output_dir = "src/flask_app/static/images/"
image_received = False
def save_image_as_jpeg(image_data):
    try:
        # Convert ROS Image message to OpenCV format
        # np_arr = np.frombuffer(image_data.data, np.uint8)
        # cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv_image = bridge.imgmsg_to_cv2(image_data, desired_encoding="bgr8")

        # Define paths for current and previous images
        current_image_path = os.path.join(output_dir, "current.jpg")
        previous_image_path = os.path.join(output_dir, "previous.jpg")
        initial_image_path = os.path.join(output_dir, "initial.jpg")

        # If current.jpg exists, move it to previous.jpg
        if os.path.exists(current_image_path):
            os.rename(current_image_path, previous_image_path)

        # Save the new image as current.jpg
        cv2.imwrite(current_image_path, cv_image)
         # Create initial.jpg if it doesn't exist
        if not os.path.exists(initial_image_path):
            cv2.imwrite(initial_image_path, cv_image)
            print("Initial image saved")


        print("images updated")

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
    # rospy.Subscriber('/xtion/rgb/image_raw/compressed', Image, image_callback)
    rospy.Subscriber('/xtion/rgb/image_raw', Image, image_callback)

    print("Waiting for images...")
    rospy.spin()

if __name__ == '__main__':
    main()
