import base64
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
import rospy

def get_image_data():
    rospy.init_node('remote_camera_reader', anonymous=True)

    def image_callback(msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Encode the image as base64 for transmission over SSH
        ret, buffer = cv2.imencode('.jpg', image)
        if ret:
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            print(image_base64)

    rospy.Subscriber("/xtion/rgb/image_raw/compressed", CompressedImage, image_callback)

    # Keep the node alive to receive images
    rospy.spin()

if __name__ == "__main__":
    get_image_data()
