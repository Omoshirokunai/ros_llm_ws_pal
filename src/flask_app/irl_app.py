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

# region LLM control

@app.route('/send_llm_prompt', methods=['POST'])
def send_llm_prompt():
    prompt = request.form.get('prompt')
    if not prompt:
        return render_template('irl_index.html',
                             error="Please enter a prompt")

    try:
        # Log the user prompt
        print(f"Received prompt: {prompt}")

        # Generate subgoals
        subgoals = llm_controller.generate_subgoals(prompt)

        # Log the response
        print(f"Generated subgoals: {subgoals}")

        if not subgoals:
            return render_template('irl_index.html',
                                error="Failed to generate subgoals",
                                user_prompt=prompt)

        #!!process subgoals and control the robot
        success = process_subgoals(prompt, subgoals)

        return render_template('irl_index.html',
                             user_prompt=prompt,
                             subgoals=subgoals, success=success)

    except Exception as e:
        print(f"Error in send_llm_prompt: {str(e)}")
        return render_template('irl_index.html',
                             error=str(e),
                             user_prompt=prompt)

# Todo: Process subgoals and send to robot
def process_subgoals(prompt, subgoals):
    """Process the subgoals and send to robot"""
    try:
        current_subgoal_index = 0
        print(f"Processing subgoals: {subgoals}, lenght: {len(subgoals)}")
        while current_subgoal_index < len(subgoals):

            # Get current images
            with open('src/flask_app/static/images/current.jpg', 'rb') as f:
                current_image = f.read()
            with open('src/flask_app/static/images/map.jpg', 'rb') as f:
                map_image = f.read()

            current_subgoal = subgoals[current_subgoal_index]
             # Get control action
            control_response = llm_controller.control_robot(current_subgoal, current_image, map_image)

            if validate_control_response(control_response):
                # Save image before action
                with open('src/flask_app/static/images/previous.jpg', 'wb') as f:
                    previous_image = f.write(current_image)

                    # f.write(current_image)

                # Execute robot action
                execute_robot_action(control_response)

                # Get new image after action
                with open('src/flask_app/static/images/current.jpg', 'rb') as f:
                    current_image = f.read()

                # Get feedback
                feedback = llm_controller.get_feedback(current_image, previous_image)

                if feedback == "continue":
                    current_subgoal_index += 1
                elif feedback == "subtask complete":
                    current_subgoal_index += 1
                elif feedback == "no progress":
                    # try alternative action
                    continue
                elif feedback == "main goal complete":
                    return True
                else:
                    print(f"Invalid control response: {control_response}")
                    continue

    except Exception as e:
        print(f"Error processing subtasks: {e}")
        return False

    return True

def validate_control_response(response):
    """Validate that control response is one of allowed actions"""
    valid_actions = [
        "move forward",
        "move backward",
        "turn left",
        "turn right",
    ]
    # check whole response if it is in the valid actions
    # return response in valid_actions
    return response.lower() in valid_actions

def execute_robot_action(action):
    """Execute robot action based on the response"""
    if action == "move forward":
        robot_control.move_forward()
    elif action == "move backward":
        robot_control.move_backward()
    elif action == "turn left":
        robot_control.turn_left()
    elif action == "turn right":
        robot_control.turn_right()
    else:
        print(f"Invalid action: {action}")

#TODO: check lidar if there is an obstacle and the next commad is to move {direction of obstacle} reprompt the llm telling it that there is an obstacle
# endregion

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001, debug=True)