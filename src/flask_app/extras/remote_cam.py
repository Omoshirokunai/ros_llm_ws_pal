#! /usr/bin/python

import os
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import sys

bridge = CvBridge()
output_dir = "/home/pal/catkin_ws/src/get_tiago_camera/src/"
image_received = False

def save_image_as_jpeg(image_data):
    print("writing image")
    global image_received

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

        print 'Image saved as %s' % (current_image_path)
    except Exception as e:
        print 'Failed to save image:%s' % (e)

def image_callback(msg):
    print("image callback")
    save_image_as_jpeg(msg)
    rospy.sleep(0.4) #!! sleep to make sure image is saved

def main():
    rospy.init_node('image_saver_node')

    # Subscribe to the camera topic
    print("sub to image")
    rospy.Subscriber('/xtion/rgb/image_raw', Image, image_callback)

    print("Waiting for images...")
    rospy.spin()

if __name__ == '__main__':
    main()
