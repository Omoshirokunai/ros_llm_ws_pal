import base64
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from io import BytesIO
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
from PIL import Image
from robot_control_over_ssh import RobotControl
from sensor_data import fetch_image_via_ssh, trigger_capture_script, trigger_stop_script

# from sensor_data import RobotSensors
from werkzeug.exceptions import HTTPException

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
def camera_worker():
    while True:
        success = fetch_image_via_ssh()
        time.sleep(12)

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

# def fetch_images_continuously():
#     while True:
#         fetch_image_via_ssh()
#         time.sleep(12)
@app.route('/video_feed')
def video_feed():
    # fetch_image_via_ssh()

    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
# endregion

# region robot control
@app.route('/robot_set_home',  methods=['POST'])
def robot_set_home():
    robot_control.robot_set_home()
    return jsonify({"status": "success", "action": "robot_set_home"})

@app.route('/pre_grasp',  methods=['POST'])
def pre_grasp():
    robot_control.pre_grasp()
    return jsonify({"status": "success", "action": "camera_feed"})
def head_up():
    print("Waiting for images...")
@app.route('/move_forward', methods=['POST'])
def move_forward():
    print("moving")
    robot_control.move_forward()
    return jsonify({"status": "success", "action": "move_forward"})

@app.route('/turn_right', methods=['POST'])
def turn_right():
    robot_control.turn_right()
    return jsonify({"status": "success", "action": "turn_right"})

@app.route('/turn_left', methods=['POST'])
def turn_left():
    robot_control.turn_left()
    return jsonify({"status": "success", "action": "turn_left"})
# endregion

# region LLM stuff
def llm_worker():
    while True:
        if not command_queue.empty():
            command = command_queue.get()
            execute_llm_command(command)
        time.sleep(0.1)

def execute_llm_command(prompt):
    try:
        # Generate subgoals using LLMController
        subgoals = llm_controller.generate_subgoals(prompt)
        if not subgoals:
            rich.print("[red]Failed to generate subgoals[/red]")
            return

        rich.print(f"[green]Generated subgoals:[/green] {subgoals}")

        for subgoal in subgoals:
            try:
                # Read current and previous images
                with open(getenv("LOCAL_IMAGE_PATH"), 'rb') as current_image_file:
                    current_image = current_image_file.read()
                with open(path.join(path.dirname(getenv("LOCAL_IMAGE_PATH")), "previous.jpg"), 'rb') as previous_image_file:
                    previous_image = previous_image_file.read()

                # Get robot action from LLMController
                action = llm_controller.control_robot(subgoal, current_image, previous_image)
                if action == "failed to understand":
                    rich.print(f"[yellow]Failed to understand subgoal:[/yellow] {subgoal}")
                    continue

                rich.print(f"[blue]Executing action:[/blue] {action}")

                                # Execute robot action
                if hasattr(robot_control, action):
                    getattr(robot_control, action)()

                # Get feedback
                feedback = llm_controller.get_feedback(current_image, previous_image)
                rich.print(f"[purple]Feedback:[/purple] {feedback}")

                if feedback == "main goal complete":
                    break

            except Exception as e:
                rich.print(f"[red]Error processing subgoal:[/red] {str(e)}")
                continue

    except Exception as e:
        rich.print(f"[red]Error in execute_llm_command:[/red] {str(e)}")


@app.route('/send_prompt', methods=['POST'])
def send_prompt():
    data = request.get_json()
    prompt = data.get('prompt')
    command_queue.put(prompt)
    return jsonify({"status": "success", "message": "Command queued"})

#endregion




if __name__ == '__main__':
    # threading.Thread(target=generate_frames, daemon=True).start()
    # threading.Thread(target=fetch_images_continuously, daemon=True).start()
    # Start camera thread
    camera_thread = threading.Thread(target=camera_worker, daemon=True)
    camera_thread.start()

    # Start LLM thread
    llm_thread = threading.Thread(target=llm_worker, daemon=True)
    llm_thread.start()
    app.run(host='0.0.0.0', port=5001, debug=True)