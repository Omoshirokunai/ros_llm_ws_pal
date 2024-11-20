import base64
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from os import getenv, path
from queue import Queue

import rich
from dotenv import load_dotenv

# import cv2
from flask import (
    Flask,
    Response,
    json,
    jsonify,
    redirect,
    render_template,
    request,
    send_file,
    url_for,
)

# from LLM_robot_control_models import control_robot, generate_subgoals, get_feedback
from LLM_robot_control_models import LLMController

# from mapping import OccupancyMapper
from PIL import Image
from robot_control_over_ssh import RobotControl
from sensor_data import trigger_capture_script, trigger_stop_script

app = Flask(__name__)
robot_control = RobotControl()
llm_controller = LLMController()

# Thread management
executor = ThreadPoolExecutor(max_workers=3)
command_queue = Queue()
camera_thread = None
llm_thread = None


# region home
@app.route('/')
def index():
    return render_template('irl_index.html')
# endregion


# Add route for map visualization
@app.route('/map_feed')
def map_feed():
    return send_file('static/images/map.jpg', mimetype='image/jpeg')

#endregion

# region camera
# Route to trigger the camera capture script if not running
@app.route('/start_capture', methods=['GET', 'POST'])
def start_capture():
    success = trigger_capture_script()

    if success:
        return "Camera capture started or already running."
    else:
        return "Failed to start camera capture."
# Route to stop the camera capture script
@app.route('/stop_capture', methods=['GET', 'POST'])
def stop_capture():
    success = trigger_stop_script()
    if success:
        return "Camera capture stopped."
    else:
        return "Failed to stop camera capture."

latest_frame = None
frame_lock = threading.Lock()


def generate_frames():

    load_dotenv()
    LOCAL_IMAGE_PATH = getenv("LOCAL_IMAGE_PATH")
    while True:
        with frame_lock:

            with open(LOCAL_IMAGE_PATH, 'rb') as image_file:
                # while True:
                image = image_file.read()
                yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + image + b'\r\n\r\n')
        time.sleep(12)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
# endregion

# region robot control
@app.route('/robot_set_home',  methods=['POST', 'GET'])
def robot_set_home():
    robot_control.robot_set_home()
    # return jsonify({"status": "success", "action": "robot_set_home"})
    return '', 204

# def head_up():
#     print("Waiting for images...")
@app.route('/move_forward', methods=['POST', 'GET'])
def move_forward():
    robot_control.move_forward()
    return '', 204

    # return jsonify({"status": "success", "action": "move_forward"})

@app.route('/turn_right', methods=['POST', 'GET'])
def turn_right():
    robot_control.turn_right()
    # return jsonify({"status": "success", "action": "turn_right"})
    return '', 204


@app.route('/turn_left', methods=['POST', 'GET'])
def turn_left():
    robot_control.turn_left()
    # return jsonify({"status": "success", "action": "turn_left"})
    return '', 204

# endregion



if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001, debug=True)