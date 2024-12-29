# sim_app_v2.py

import base64
import os

# Add signal handler for clean shutdown
import signal
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from queue import Queue

import cv2
import rich
import rospy
from cv_bridge import CvBridge, CvBridgeError
from evaluation_metrics import ExperimentLogger
from flask import Flask, Response, jsonify, redirect, render_template, request, url_for
from lidar_safety import LidarSafety
from LLM_robot_control_models import LLMController
from sensor_msgs.msg import Image, LaserScan
from sim_robot_control import move_forward_by, move_robot, turn_by_angle

# Initialize Flask app
app = Flask(__name__)

# Initialize components
llm_controller = LLMController()
bridge = CvBridge()
lidar_safety = LidarSafety()
experiment_logger = ExperimentLogger()
# evaluator = TaskEvaluator()

# Thread management
executor = ThreadPoolExecutor(max_workers=3)
command_queue = Queue()
camera_thread = None
llm_thread = None
stop_llm = False


# Image paths
IMAGE_DIR = 'src/flask_app/static/images'
IMAGE_PATHS = {
    'initial': os.path.join(IMAGE_DIR, 'initial.jpg'),
    'current': os.path.join(IMAGE_DIR, 'current.jpg'),
    'previous': os.path.join(IMAGE_DIR, 'previous.jpg'),
    'map': os.path.join(IMAGE_DIR, 'sim_map.jpg')
}

# Global variables
latest_frame = None
frame_lock = threading.Lock()
VALID_ACTIONS = ["move forward", "move backward", "turn left", "turn right"]

# Ensure image directory exists
os.makedirs(IMAGE_DIR, exist_ok=True)

# Region: Basic Routes
@app.route('/')
def index():
    return render_template('sim_index.html')

# Region: Camera Handling
def image_callback(msg):
    global latest_frame
    try:
        with frame_lock:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            _, jpeg = cv2.imencode('.jpg', cv_image)
            latest_frame = jpeg.tobytes()
            # Save for LLM processing
            with open('static/images/current.jpg', 'wb') as f:
                f.write(latest_frame)
    except CvBridgeError as e:
        print(e)

def generate_frames():
    while True:
        try:
            with open(IMAGE_PATHS['current'], 'rb') as f:
                frame = f.read()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        except Exception as e:
            rich.print(f"[red]Error generating frames:[/red] {str(e)}")
        time.sleep(0.2)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

# Region: Robot Control
@app.route('/move_forward', methods=['POST', 'GET'])
def move_forward():
    move_robot('forward')
    return '', 204

@app.route('/move_backward', methods=['POST', 'GET'])
def move_backward():
    move_robot('backward')
    return '', 204

@app.route('/turn_left', methods=['POST', 'GET'])
def turn_left():
    move_robot('left')
    return '', 204

@app.route('/turn_right', methods=['POST', 'GET'])
def turn_right():
    move_robot('right')
    return '', 204

# Region: LLM Control
def validate_control_response(response):
    """Validate that control response is one of allowed actions"""
    return response and response.lower() in VALID_ACTIONS

def execute_robot_action(action):
    """Execute robot action based on the response"""
    action = action.lower()
    if action == "move forward":
        return move_robot('forward')
    elif action == "move backward":
        return move_robot('backward')
    elif action == "turn left":
        return move_robot('left')
    elif action == "turn right":
        return move_robot('right')

    # # Handle parameterized commands
    # parts = action.split()
    # if len(parts) >= 6:
    #     action = " ".join(parts[0:2])
    #     value = float(parts[2])
    #     speed = float(parts[5].rstrip("m/s"))

    #     if action == "move forward" and "meters" in response:
    #         return move_forward_by(value, speed)
    #     elif action == "turn left" and "degrees" in response:
    #         return turn_by_angle(value, speed)
    #     elif action == "turn right" and "degrees" in response:
    #         return turn_by_angle(-value, speed)

    # return False, "Invalid command format"

    # except Exception as e:
    #     return False, f"Action failed: {str(e)}"
    return False


def save_current_frame(frame_data):
    """Save current frame and handle image rotation"""
    try:
        # Save as current
        with open(IMAGE_PATHS['current'], 'wb') as f:
            f.write(frame_data)

        # If initial doesn't exist, create it
        if not os.path.exists(IMAGE_PATHS['initial']):
            with open(IMAGE_PATHS['initial'], 'wb') as f:
                f.write(frame_data)

        # Move current to previous if previous doesn't exist
        if not os.path.exists(IMAGE_PATHS['previous']):
            with open(IMAGE_PATHS['current'], 'rb') as src:
                with open(IMAGE_PATHS['previous'], 'wb') as dst:
                    dst.write(src.read())

    except Exception as e:
        rich.print(f"[red]Error saving frame: {e}[/red]")

@app.route('/send_llm_prompt', methods=['POST'])
def send_llm_prompt():

    try:
        prompt = request.form.get('prompt')
        if not prompt:
            return render_template('sim_index.html',
                                error="Please enter a prompt")

        rich.print(f"[blue]Received prompt:[/blue] {prompt}")

        # Get initial image
        images = load_images()
        if os.path.exists(IMAGE_PATHS['initial']):
            os.remove(IMAGE_PATHS['initial'])

        # if not images:
        #     return render_template('sim_index.html',
        #                         error="Failed to load images",
        #                         user_prompt=prompt)

        # Generate subgoals with initial image context
        subgoals = llm_controller.generate_subgoals(prompt, images['current'])

        if not subgoals:
            return render_template('sim_index.html',
                                error="Failed to generate subgoals",
                                user_prompt=prompt)

        success = process_subgoals(prompt, subgoals)
        return render_template('sim_index.html',
                             user_prompt=prompt,
                             subgoals=subgoals,
                             success=success)

    except Exception as e:
        rich.print(f"[red]Error in send_llm_prompt:[/red] {str(e)}")
        return render_template('sim_index.html',
                             error=str(e),
                             user_prompt=prompt)

# def save_current_frame(frame_data):
#     """Save current frame"""
#     try:
#         with open(IMAGE_PATHS['current'], 'wb') as f:
#             f.write(frame_data)
#     except Exception as e:
#         rich.print(f"[red]Error saving frame:[/red] {str(e)}")

# def image_callback(msg):
#     """Handle incoming camera frames"""
#     try:
#         with frame_lock:
#             cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
#             save_current_frame(cv_image)
#     except Exception as e:
#         rich.print(f"[red]Error in image callback:[/red] {str(e)}")

def image_callback(msg):
    """Handle incoming camera frames"""
    global latest_frame
    try:
        with frame_lock:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            _, jpeg = cv2.imencode('.jpg', cv_image)
            frame_data = jpeg.tobytes()
            latest_frame = frame_data
            save_current_frame(frame_data)
            rospy.sleep(0.2)
            # rospy.signal_shutdown('Image captured, shutting down node.')

    except CvBridgeError as e:
        rich.print(f"[red]Camera error: {e}[/red]")

# def load_images():
#     """Load all required images"""
#     images = {}
#     try:
#         for img_type, path in IMAGE_PATHS.items():
#             if os.path.exists(path):
#                 with open(path, 'rb') as f:
#                     images[img_type] = f.read()
#             else:
#                 rich.print(f"[yellow]Warning: {img_type} image not found[/yellow]")
#                 return None
#         return images
#     except Exception as e:
#         rich.print(f"[red]Error loading images: {e}[/red]")
#         return None

def load_images():
    """Load all required images"""
    images = {}
    try:
        for img_type, path in IMAGE_PATHS.items():
            with open(path, 'rb') as f:
                images[img_type] = f.read()
        return images
    except Exception as e:
        rich.print(f"[red]Error loading images:[/red] {str(e)}")
        return None

def process_subgoals(prompt, subgoals):
    """Process subgoals for simulation"""
    current_subgoal_index = 0
    executed_actions = []
    last_feedback = None
    initial_image = IMAGE_PATHS['initial']

    global stop_llm
    stop_llm = False

    try:
        # Start logging session
        session_id = experiment_logger.start_session(prompt, llm_controller.model_name)
        experiment_logger.log_subgoals(subgoals, time.time() - experiment_logger.session_start_time)

        while current_subgoal_index < len(subgoals):
            if stop_llm:
                experiment_logger.complete_session(False, time.time() - experiment_logger.session_start_time)
                return False

            current_subgoal = subgoals[current_subgoal_index].split(" ", 1)[1]

            # Get safety context
            safety_context = lidar_safety.get_safety_context()

            # Get control action
            control_response = llm_controller.control_robot(
                subgoal=current_subgoal,
                initial_image=initial_image,
                current_image=IMAGE_PATHS['current'],
                previous_image=IMAGE_PATHS['previous'],
                map_image=IMAGE_PATHS['map'],
                executed_actions=executed_actions,
                last_feedback=last_feedback,
                all_subgoals=subgoals,
                safety_warning=None,
                safety_context=safety_context
            )

            # Validate and execute control response
            if not validate_control_response(control_response):
                experiment_logger.log_invalid_control(current_subgoal, control_response)
                last_feedback = f"{control_response} is an invalid action"
                continue

            # Check safety
            is_safe, warning = lidar_safety.check_direction_safety(control_response)
            if not is_safe:
                experiment_logger.log_safety_trigger(current_subgoal, warning)
                last_feedback = f"Safety warning: {warning}"
                continue

            # Execute action and log
            success = execute_robot_action(control_response, current_subgoal)
            if success:
                executed_actions.append(control_response)
                experiment_logger.log_action(current_subgoal, control_response)
            else:
                experiment_logger.log_invalid_control(current_subgoal, control_response)
                last_feedback = "Failed to execute action"
                continue

            # Get feedback
            feedback = llm_controller.get_feedback(
                initial_image=initial_image,
                current_image=IMAGE_PATHS['current'],
                previous_image=IMAGE_PATHS['previous'],
                map_image=IMAGE_PATHS['map'],
                current_subgoal=current_subgoal,
                executed_actions=executed_actions,
                last_feedback=last_feedback,
                subgoals=subgoals
            )

            experiment_logger.log_feedback(current_subgoal, feedback)
            last_feedback = feedback

            # Process feedback
            if "subtask complete" in feedback:
                current_subgoal_index += 1
                executed_actions = []
                last_feedback = None
            elif "main goal complete" in feedback:
                experiment_logger.complete_session(True, time.time() - experiment_logger.session_start_time)
                return True

        success = current_subgoal_index >= len(subgoals)
        experiment_logger.complete_session(success, time.time() - experiment_logger.session_start_time)
        return success

    except Exception as e:
        print(f"Error in process_subgoals: {str(e)}")
        return False

def validate_control_response(response):
    """Validate control response"""
    # valid_actions = ["move forward", "move backward", "turn left", "turn right", "completed"]
    # return response and response.lower() in valid_actions
    basic_actions = {
        "move forward": True,
        "move backward": True,
        "turn left": True,
        "turn right": True,
        "completed": True
    }

    response = response.lower().strip()
    if response in basic_actions:
        return True

    # Parse parameterized commands
    try:
        parts = response.split()
        if len(parts) >= 6:
            action = " ".join(parts[0:2])
            value = float(parts[2])
            speed = float(parts[5].rstrip("m/s"))

            if action == "move forward" and "meters" in response:
                if 0.1 <= value <= 2.0 and 0.1 <= speed <= 0.5:
                    return True
    except:
        pass

    return False

def execute_robot_action(action):
    """Execute robot action in simulation"""
    try:

        response = action.lower().strip()
         # Check safety first
        is_safe, warning = lidar_safety.check_direction_safety(response)
        if not is_safe:
            return False

        if action == "move forward":
            move_robot('forward')
        elif action == "move backward":
            move_robot('backward')
        elif action == "turn left":
            move_robot('left')
        elif action == "turn right":
            move_robot('right')
        elif action == "completed":
            return True
        else:
            return False

        # rospy.sleep(0.1)  # Allow time for action
        # return True

        # Parameterized commands
        parts = response.split()
        if len(parts) >= 6:
            action = " ".join(parts[0:2])
            value = float(parts[2])
            speed = float(parts[5].rstrip("m/s"))

            if action == "move forward" and "meters" in response:
                return move_forward_by(value, speed)
            elif action == "turn left" and "degrees" in response:
                return turn_by_angle(value, speed)
            elif action == "turn right" and "degrees" in response:
                return turn_by_angle(-value, speed)

        return False

    except Exception as e:
        rich.print(f"[red]Action failed:[/red] {str(e)}")
        return False

# Routes and other functions remain mostly unchanged...

rospy.Subscriber("/xtion/rgb/image_raw", Image, image_callback)



# signal.signal(signal.SIGINT, signal_handler)

@app.route('/stop_llm', methods=['POST'])
def stop_llm_control():
    """Stop LLM control and generate evaluation report"""
    global stop_llm
    stop_llm = True

    return redirect(url_for('index'))

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
