# region Imports
from flask import Flask, jsonify, request, Response, render_template, redirect, url_for
import rospy # type: ignore #* ROS Python client library
from sensor_msgs.msg import Image, LaserScan, JointState# type: ignore #* camera and lidar data
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
import google.api_core.exceptions
from rich import print as rprint

# endregion Imports

# region Flask and ROS config
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = CREDENTIALS
app = Flask(__name__)

bridge = CvBridge()

#* Global variables to store latest camera frame and lidar data
latest_frame = None
lidar_data = None
current_arm_position = None
gemini_response = None
stop_gemini = False
frame_lock = threading.Lock() # protect process from being accessed by multiple threads at the same time
lidar_lock = threading.Lock()
arm_position_lock = threading.Lock()
gemini_response_lock = threading.Lock()


# endregion

# region Home
# #* Home
@app.route('/')
def index():
    return render_template('index.html', response="", arm_positions=ARM_POSITIONS, current_position=current_position)

# endregion

# region Gemini
# region Gemini_init
vertexai.init(project=PROJECT_ID, location=REGIONEU)
# endregion gemini_init

def get_camera_image():
    global latest_frame
    with frame_lock:
        if latest_frame is not None:
            _, buffer = cv2.imencode('.jpg', latest_frame)
            image_str = buffer.tobytes()
            return image_str
    return None
def multiturn_generate_content(system_prompt, message="", image=None, generation_config=None, safety_settings=None):
    generative_multimodal_model = GenerativeModel("gemini-1.5-pro", system_instruction=[system_prompt])
    try:
        chat = generative_multimodal_model.start_chat()
        api_response = chat.send_message([image, message], generation_config=generation_config, safety_settings=safety_settings)
        return api_response
    except google.api_core.exceptions.ResourceExhausted:
        print("ResourceExhausted")
        return None
    except vertexai.generative_models._generative_models.ResponseValidationError:
        print("ResponseValidationError")
        return None

def gemini_control_loop(prompt):
    global gemini_response, stop_gemini
    stop_gemini = False
    while not stop_gemini:
        image_str = get_camera_image()
        if image_str is None:
            continue

        image = GeminiImage.from_bytes(image_str)
        response = multiturn_generate_content(system_prompt, message=prompt, image=image, generation_config=generation_config, safety_settings=safety_settings)

        if response and response.candidates:
            with gemini_response_lock:
                gemini_response = response.candidates[0].text

                if gemini_response.startswith("move"):
                    direction = gemini_response.split(" ")[1]

                    move_robot(direction)
                elif gemini_response.startswith("update_arm"):
                    position_name = gemini_response.split(" ")[1]
                    if position_name in ARM_POSITIONS:
                        update_arm(ARM_POSITIONS[position_name])

                if "done" in gemini_response.lower():
                    stop_gemini = True

        rospy.sleep(1)

@app.route('/send_prompt', methods=['POST'])
def send_prompt():
    global gemini_response
    user_prompt = request.form.get('prompt')

    gemini_thread = threading.Thread(target=gemini_control_loop, args=(user_prompt,))
    gemini_thread.start()

    return render_template('index.html', response=gemini_response, arm_positions=ARM_POSITIONS.keys(), current_position=get_current_arm_position_name(current_arm_position))

@app.route('/stop_gemini', methods=['POST'])
def stop_gemini_control():
    global stop_gemini
    stop_gemini = True
    return redirect(url_for('index'))
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
        rospy.sleep(1)

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
# [arm_1_joint = base joint
# arm_2_joint= sholder joint
# , arm_3_joint= rotate whole arm
# , arm_4_joint= elbow
# , arm_5_joint= rotate wrist
# , arm_6_joint= wrist move forward and backward
# , arm_7_joint = rotate wrsit ]
ARM_POSITIONS = {
    # Grabbing positions
    'arm_left_point_up': [0.08, 0.07, -3.0, 1.5, -1.57, 0.2, 0.0],
    'pre_grab': [0.08, 0.8, -1.7, 1.5, 0.0, 0.2, 0.0],
    'reach_forward': [1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5],
}
#function to move arm_3_joint forward by adding to the current position
def move_arm_3_joint_forward():
    global current_arm_position
    if current_arm_position is None:
        return
    current_arm_position[2] += 0.1
    update_arm(current_arm_position)

def joint_state_callback(msg):
    global current_arm_position
    arm_joints = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
    new_position = [msg.position[msg.name.index(joint)] for joint in arm_joints]

    with arm_position_lock:
        current_arm_position = new_position
# Subscribe to joint states
rospy.Subscriber('/joint_states', JointState, joint_state_callback)

def get_current_arm_position_name(current_arm_position):
    with arm_position_lock:
        if current_arm_position is None:
            return "Unknown"
        for name, positions in ARM_POSITIONS.items():
            if all(abs(a - b) < 0.1 for a, b in zip(current_arm_position, positions)):
                return name
    return "Custom"

current_position = get_current_arm_position_name(current_arm_position)

@app.route('/update_arm', methods=['POST'])
def handle_update_arm():
    global current_position
    position_name = request.form['arm_position']
    if position_name in ARM_POSITIONS:
        update_arm(ARM_POSITIONS[position_name])
        current_position = position_name
    return redirect(url_for('index'))

# endregion arm
# endregion Robot Control


# region main
if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')

# endregion main