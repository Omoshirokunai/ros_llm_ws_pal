# sim_app_v2.py

import base64
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from queue import Queue
from evaluation_metrics import TaskEvaluator
from lidar_safety import LidarSafety
import cv2
import rich
import rospy
from cv_bridge import CvBridge, CvBridgeError
from flask import Flask, Response, jsonify, render_template, request
from LLM_robot_control_models import LLMController
from sensor_msgs.msg import Image, LaserScan
from sim_robot_control import move_robot

# Initialize Flask app
app = Flask(__name__)

# Initialize components
llm_controller = LLMController()
bridge = CvBridge()
lidar_safety = LidarSafety()
evaluator = TaskEvaluator()

# Thread management
executor = ThreadPoolExecutor(max_workers=3)
command_queue = Queue()
camera_thread = None
llm_thread = None

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
    return False


# @app.route('/send_llm_prompt', methods=['POST'])
# def send_llm_prompt():
#     prompt = request.form.get('prompt')
#     if not prompt:
#         return render_template('sim_index.html',
#                              error="Please enter a prompt")

#     try:
#         rich.print(f"[blue]Received prompt:[/blue] {prompt}")
#         subgoals = llm_controller.generate_subgoals(prompt)

#         if not subgoals:
#             return render_template('sim_index.html',
#                                 error="Failed to generate subgoals",
#                                 user_prompt=prompt)

#         success = process_subgoals(prompt, subgoals)
#         return render_template('sim_index.html',
#                              user_prompt=prompt,
#                              subgoals=subgoals,
#                              success=success)

#     except Exception as e:
#         rich.print(f"[red]Error in send_llm_prompt:[/red] {str(e)}")
#         return render_template('sim_index.html',
#                              error=str(e),
#                              user_prompt=prompt)

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
    """Process subgoals for simulation robot"""
    current_subgoal_index = 0
    executed_actions = []
    last_feedback = None
    initial_image = IMAGE_PATHS['initial']

    # Start evaluation
    evaluator.start_task(prompt, subgoals)
    try:
        # Get initial state
        images = load_images()
        if not images:
            raise Exception("Failed to load initial images")

        # Save initial state
        # with open(IMAGE_PATHS['current'], 'rb') as src:
        #     with open(initial_image, 'wb') as dst:
        #         dst.write(src.read())
        if not os.path.exists(IMAGE_PATHS['initial']):
            with open(IMAGE_PATHS['current'], 'rb') as src:
                with open(IMAGE_PATHS['initial'], 'wb') as dst:
                    dst.write(src.read())

        rich.print(f"[blue]Processing {len(subgoals)} subgoals for goal:[/blue] {prompt}")

        while current_subgoal_index < len(subgoals):
            current_subgoal = subgoals[current_subgoal_index].split(" ", 1)[1]

            # if current_subgoal.startswith("stop"):
            #     rich.print(f"[green]Task completed successfully[/green]")
            #     return True
            if current_subgoal.startswith("stop"):
                rich.print(f"[red]Stopping task:[/red] {current_subgoal}")
                evaluator.generate_report("src/flask_app/static/evaluation_results")
                return True

            rich.print(f"\n[cyan]Current subgoal ({current_subgoal_index + 1}/{len(subgoals)}):[/cyan] {current_subgoal}")

            # Get fresh images
            images = load_images()
            if not images:
                continue

            # Get control action
            control_response = llm_controller.control_robot(
                subgoal=current_subgoal,
                initial_image=images['initial'],
                current_image=images['current'],
                previous_image=images['previous'],
                map_image=images['map'],
                executed_actions=executed_actions,
                last_feedback=last_feedback,
                all_subgoals=subgoals
            )

            if not validate_control_response(control_response):
                rich.print(f"[red]Invalid control response:[/red] {control_response}")
                continue

            # Save current as previous before executing action
            with open(IMAGE_PATHS['current'], 'rb') as src:
                with open(IMAGE_PATHS['previous'], 'wb') as dst:
                    dst.write(src.read())

            # Check safety before execution
            is_safe, warning = lidar_safety.check_direction_safety(control_response)
            evaluator.log_safety_event(True, not is_safe)

            if not is_safe:
                rich.print(f"[red]Safety check failed:[/red] {warning}")
                last_feedback = f"Safety warning: {warning}"
                continue

            # Execute action
            if execute_robot_action(control_response):
                evaluator.log_action(control_response) #evaluator log action
                executed_actions.append(control_response)
                rich.print(f"[green]Executed action:[/green] {control_response}")
                time.sleep(0.1)
            else:
                rich.print("[red]Failed to execute robot action[/red]")
                continue

            # Get fresh images for feedback
            images = load_images()
            if not images:
                continue

            # Get feedback
            feedback = llm_controller.get_feedback(
                initial_image=images['initial'],
                current_image=images['current'],
                previous_image=images['previous'],
                map_image=images['map'],
                current_subgoal=current_subgoal,
                executed_actions=executed_actions,
                last_feedback=last_feedback,
                subgoals=subgoals
            )

            rich.print(f"[purple]Feedback received:[/purple] {feedback}")

            # Process feedback
            if feedback == "continue":
                continue
            elif feedback == "subtask complete":
                current_subgoal_index += 1
                executed_actions = []
                last_feedback = None
            elif feedback == "main goal complete":
                return True
            elif feedback == "no progress":
                last_feedback = "Previous action made no progress, try a different approach"
                continue
            elif feedback.startswith("do"):
                last_feedback = feedback
                continue

        # return current_subgoal_index >= len(subgoals)
        # Task completed
        success = current_subgoal_index >= len(subgoals)
        evaluator.complete_task(success)
        evaluator.generate_report("static/evaluation_results")
        return success

    except Exception as e:
         # Ensure evaluation is saved even on error
        evaluator.complete_task(False)
        evaluator.generate_report("static/evaluation_results")

        rich.print(f"[red]Error in process_subgoals:[/red] {str(e)}")
        return False


def validate_control_response(response):
    """Validate control response"""
    valid_actions = ["move forward", "move backward", "turn left", "turn right", "completed"]
    return response and response.lower() in valid_actions

def execute_robot_action(action):
    """Execute robot action in simulation"""
    try:

         # Check safety first
        is_safe, warning = lidar_safety.check_direction_safety(action)
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

        rospy.sleep(0.1)  # Allow time for action
        return True
    except Exception as e:
        rich.print(f"[red]Action failed:[/red] {str(e)}")
        return False

# Routes and other functions remain mostly unchanged...

rospy.Subscriber("/xtion/rgb/image_raw", Image, image_callback)

# Add signal handler for clean shutdown
import signal
import sys

def signal_handler(sig, frame):
    """Ensure evaluation is saved on exit"""
    if evaluator.current_task:
        evaluator.complete_task(False)
        evaluator.generate_report("src/flask_app/static/evaluation_results")
    rich.print("[yellow]Shutting down...[/yellow]")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
