# region Imports
from flask import Flask, jsonify, request, Response, render_template, redirect, url_for
import rospy # type: ignore #* ROS Python client library
from sensor_msgs.msg import Image, LaserScan # type: ignore #* camera and lidar data
import cv2
from cv_bridge import CvBridge, CvBridgeError # type: ignore #* convert ROS messages to OpenCV images
import threading
import vertexai
from vertexai.preview.generative_models import GenerativeModel
from vertexai.preview.generative_models import Image as GeminiImage
from safe import PROJECT_ID, REGIONEU, CREDENTIALS
from gemini_config import generation_config, safety_settings, system_prompt
import os
from robot_control import move_robot, update_torso, update_arm
# endregion Imports

# region Flask and ROS config
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = CREDENTIALS
app = Flask(__name__)

bridge = CvBridge()

#* Global variables to store latest camera frame and lidar data
latest_frame = None
lidar_data = None
frame_lock = threading.Lock() # protect process from being accessed by multiple threads at the same time
lidar_lock = threading.Lock()

# endregion

# region Home
# #* Home
@app.route('/')
def index():
    return render_template('index.html', response="", arm_positions=ARM_POSITIONS, current_position=current_position)
# @app.route('/')
# def index():
#     return render_template('index.html', response=" ")
# endregion

# region Gemini
# region Gemini_init
vertexai.init(project=PROJECT_ID, location=REGIONEU)
generative_multimodal_model = GenerativeModel("gemini-1.5-pro")
gemini_response = None
# endregion gemini_init

@app.route('/send_prompt', methods=['POST'])
def send_prompt():

    global gemini_response
    user_prompt = request.form.get('prompt')
    # print(user_prompt)

    if system_prompt and user_prompt != " ":
        prompt = f"{system_prompt}\n\n{user_prompt}"
    else:
        prompt = user_prompt

    response = generative_multimodal_model.generate_content(prompt)
    gemini_response = response.candidates[0].text

    return render_template('index.html', response = gemini_response)
    # return redirect(url_for('index', response=gemini_response , _anchor='response'))
# endregion Gemini

# region Sensors
# region camera
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
        rospy.sleep(0)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# endregion camera

# region lidar
def lidar_callback(data):
    global lidar_data
    with lidar_lock:
        lidar_data = data

# Subscribe to the LIDAR topic
rospy.Subscriber('/scan_raw', LaserScan, lidar_callback, queue_size=10)

@app.route('/lidar')
def get_lidar_data():
    """
    Route to retrieve the latest lidar data as JSON.
    """
    global lidar_data
    with lidar_lock:
        if lidar_data:
            return jsonify({
                'ranges': lidar_data.ranges,
            })
        else:
            return jsonify({'error': 'No data received'}), 500

# endregion lidar
# endregion Sensors

# region Robot Control
# region movemnt
@app.route('/move', methods=['POST'])
def move():
    """
    Handle robot movement commands based on the input from the form.
    """
    direction = request.form['direction']
    move_robot(direction)
    return '', 204

# endregion movement

# region torso
@app.route('/update_torso', methods=['POST'])
def handle_update_torso():
    """
    Update the position of the robot's torso based on the input from the form.
    """
    position = float(request.form['torso']) / 100.0  # Scale the value to 0.0 - 1.0 range
    update_torso(position)
    return '', 204

# endregion torso

# region arm
ARM_POSITIONS = {
    'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'reach_low': [0.0, -0.8, 0.0, 1.5, 0.0, -0.8, 0.0],
    'reach_low_forward': [1.5, -0.8, 0.0, 1.5, 0.0, -0.8, 1.5],
    'reach_high': [0.0, 0.8, 0.0, -1.5, 0.0, 0.8, 0.0],
    'reach_high_forward': [1.5, 0.8, 0.0, -1.5, 0.0, 0.8, 1.5],
    'wave': [0.0, -0.3, 0.0, 1.0, 0.0, -0.7, 0.0],
    'reach_forward': [1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5],
    'tuck': [0.0, 1.3, 0.0, 2.0, 0.0, 0.5, 0.0]
}

current_position = 'home'
@app.route('/update_arm', methods=['POST'])
def handle_update_arm():
    global current_position
    position_name = request.form['arm_position']
    if position_name in ARM_POSITIONS:
        update_arm(ARM_POSITIONS[position_name])
        current_position = position_name
    return redirect(url_for('index'))

# @app.route('/update_arm', methods=['POST'])
# def handle_update_arm():
#     joint_positions = []
#     for i in range(1, 8):
#         joint_name = f'arm_joint_{i}'
#         joint_positions.append(float(request.form[joint_name]) / 100.0)
#     update_arm(joint_positions)
#     return '', 204
# endregion arm
# endregion Robot Control


# region main
if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')

# endregion main