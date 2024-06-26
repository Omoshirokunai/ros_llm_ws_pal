from flask import Flask, jsonify, request, Response, render_template
import rospy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge, CvBridgeError
import threading
import vertexai
# from vertexai.preview.generative_models import GenerativeModel, Image
# from safe import PROJECT_ID, REGIONNA, CREDENTIALS
import os


#* Flask and ROS config
app = Flask(__name__)

rospy.init_node('flask_controller', anonymous=True)
cmd_vel_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
bridge = CvBridge()

latest_frame = None
lidar_data = None
frame_lock = threading.Lock()
lidar_lock = threading.Lock()
# os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = CREDENTIALS

# vertexai.init(project=PROJECT_ID, location=REGIONNA)
# generative_multimodal_model = GenerativeModel("gemini-1.5-pro")
#* Home
@app.route('/')
def index():
    return render_template('index.html')

# @app.route('/send_prompt', methods=['POST'])
# def send_prompt():
#     system_prompt = "you are tiago Pal robot in a room with a camera. You can move around and interact with the environment. "
#     user_prompt = request.form.get('prompt')
    
#     if system_prompt:
#         prompt = f"{system_prompt}\n\n{user_prompt}"
#     else:
#         prompt = user_prompt

#     response = generative_multimodal_model.generate_content(prompt)
#     return response.candidates[0].text

#* Camera
def image_callback(msg):
    global latest_frame
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # Optionally resize the image for better performance
        resized_image = cv2.resize(cv_image, (640, 480))
        with frame_lock:
            latest_frame = resized_image
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

# Subscribe to the camera topic
rospy.Subscriber('/xtion/rgb/image_raw', Image, image_callback, queue_size=10)

def generate_frames():
    global latest_frame
    while True:
        with frame_lock:
            if latest_frame is not None:
                ret, buffer = cv2.imencode('.jpg', latest_frame)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        rospy.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

#* Lidar section
def lidar_callback(data):
    global lidar_data
    with lidar_lock:
        lidar_data = data

# Subscribe to the LIDAR topic
rospy.Subscriber('/scan_raw', LaserScan, lidar_callback, queue_size=10)

@app.route('/lidar')
def get_lidar_data():
    global lidar_data
    with lidar_lock:
        if lidar_data:
            return jsonify({
                'ranges': lidar_data.ranges,
            })
        else:
            return jsonify({'error': 'No data received'}), 500

#* Movement section
@app.route('/move', methods=['POST'])
def move():
    direction = request.form['direction']
    twist = Twist()

    if direction == 'forward':
        twist.linear.x = 1
    elif direction == 'backward':
        twist.linear.x = -1
    elif direction == 'left':
        twist.angular.z = 1
    elif direction == 'right':
        twist.angular.z = -1

    cmd_vel_publisher.publish(twist)
    return '', 204

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')
