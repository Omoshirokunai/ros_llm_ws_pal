import base64

import rospy
from sensor_msgs.msg import CompressedImage


# def image_callback(msg):
#     # Convert the image data to base64
#     image_base64 = base64.b64encode(msg.data).decode('utf-8')
#     print(image_base64)  # This will be sent back to the Flask app

# rospy.init_node('camera_subscriber', anonymous=True)
# rospy.Subscriber("/xtion/rgb/image_raw/compressed", CompressedImage, image_callback)
# rospy.spin()

class CameraCapture:
    def __init__(self):
        self.image_data = None
        self.subscriber = rospy.Subscriber("/xtion/rgb/image_raw/compressed", CompressedImage, self.image_callback)
        rospy.loginfo("CameraCapture initialized.")

    def image_callback(self, msg):
        # Convert the image data to base64
        self.image_data = base64.b64encode(msg.data).decode('utf-8')

    def get_latest_image(self):
        # Wait for a moment to get the latest image
        rospy.sleep(0.1)  # Wait briefly to ensure the latest image is captured
        return self.image_data if self.image_data else None

if __name__ == "__main__":
    rospy.init_node('camera_capture_node')
    camera_capture = CameraCapture()

    # Loop to keep the script running
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        latest_image = camera_capture.get_latest_image()
        if latest_image:
            print(latest_image)  # Output base64 data
        rate.sleep()
