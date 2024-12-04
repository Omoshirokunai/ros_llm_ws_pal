# # sim_app_v2.py

# import base64
# import threading
# import time
# from queue import Queue

# import rich
# import rospy
# from cv_bridge import CvBridge, CvBridgeError
# from flask import Flask, Response, jsonify, render_template, request, url_for
# from sensor_data import fetch_images
# from sensor_msgs.msg import Image, LaserScan

# from flask_app.LLM_robot_control_models import LLMController
# from flask_app.sim_robot_control import move_robot

# # Initialize Flask app
# app = Flask(__name__)

# # Initialize components
# llm_controller = LLMController()
# bridge = CvBridge()

# # Thread management
# # executor = ThreadPoolExecutor(max_workers=3)
# command_queue = Queue()
# camera_thread = None
# llm_thread = None

# # Global variables
# latest_frame = None
# lidar_data = None
# frame_lock = threading.Lock()
# lidar_lock = threading.Lock()

# # Valid robot actions
# VALID_ACTIONS = ["move forward", "move backward", "turn left", "turn right"]

# # ROS Initialization
# rospy.init_node('sim_robot_control', anonymous=True)



# def process_subgoals(prompt, subgoals):
#     """Process subgoals for simulation robot"""
#     current_subgoal_index = 0
#     executed_actions = []
#     last_feedback = None
#     initial_image = None

#     try:
#         # Get initial state image
#         if not fetch_images():
#             raise Exception("Failed to fetch initial images")
#         with open('static/images/current.jpg', 'rb') as f:
#             initial_image = f.read()

#         rich.print(f"[blue]Processing {len(subgoals)} subgoals for goal:[/blue] {prompt}")

#         while current_subgoal_index < len(subgoals):
#             current_subgoal = subgoals[current_subgoal_index].split(" ", 1)[1]
#             rich.print(f"\n[cyan]Current subgoal ({current_subgoal_index + 1}/{len(subgoals)}):[/cyan] {current_subgoal}")

#             if not fetch_images():
#                 continue

#             try:
#                 with open('static/images/current.jpg', 'rb') as f:
#                     current_image = f.read()
#                 with open('static/images/previous.jpg', 'rb') as f:
#                     previous_image = f.read()
#                 with open('static/images/map.jpg', 'rb') as f:
#                     map_image = f.read()
#             except Exception as e:
#                 rich.print(f"[red]Error reading images: {e}[/red]")
#                 continue

#             # Get control action
#             control_response = llm_controller.control_robot(
#                 subgoal=current_subgoal,
#                 initial_image=initial_image,
#                 current_image=current_image,
#                 previous_image=previous_image,
#                 map_image=map_image,
#                 executed_actions=executed_actions,
#                 last_feedback=last_feedback
#             )

#             if not validate_control_response(control_response):
#                 rich.print(f"[red]Invalid control response:[/red] {control_response}")
#                 continue

#             # Save current as previous
#             with open('static/images/current.jpg', 'rb') as src:
#                 with open('static/images/previous.jpg', 'wb') as dst:
#                     dst.write(src.read())

#             # Execute action
#             if execute_robot_action(control_response):
#                 executed_actions.append(control_response)
#                 rich.print(f"[green]Executed action:[/green] {control_response}")
#                 time.sleep(2)
#             else:
#                 rich.print("[red]Failed to execute robot action[/red]")
#                 continue

#             # Get feedback
#             if not fetch_images():
#                 continue

#             try:
#                 with open('static/images/current.jpg', 'rb') as f:
#                     new_current_image = f.read()
#                 with open('static/images/previous.jpg', 'rb') as f:
#                     new_previous_image = f.read()
#             except Exception as e:
#                 rich.print(f"[red]Error reading feedback images: {e}[/red]")
#                 continue

#             feedback = llm_controller.get_feedback(
#                 initial_image=initial_image,
#                 current_image=new_current_image,
#                 previous_image=new_previous_image,
#                 map_image=map_image,
#                 current_subgoal=current_subgoal,
#                 executed_actions=executed_actions,
#                 last_feedback=last_feedback
#             )

#             rich.print(f"[purple]Feedback received:[/purple] {feedback}")
#             last_feedback = feedback

#             # Process feedback
#             if feedback == "continue":
#                 continue
#             elif feedback == "subtask complete":
#                 current_subgoal_index += 1
#                 executed_actions = []
#                 last_feedback = None
#             elif feedback == "main goal complete":
#                 return True
#             elif feedback == "no progress":
#                 last_feedback = "Previous action made no progress, try a different approach"
#                 continue
#             elif feedback.startswith("do"):
#                 last_feedback = feedback
#                 continue

#         return current_subgoal_index >= len(subgoals)

#     except Exception as e:
#         rich.print(f"[red]Error in process_subgoals:[/red] {str(e)}")
#         return False

# ROS Subscribers





# sim_app_v2.py

import base64
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import cv2
import rich
import rospy
from cv_bridge import CvBridge, CvBridgeError
from flask import Flask, Response, jsonify, render_template, request
from sensor_msgs.msg import Image, LaserScan

from flask_app.LLM_robot_control_models import LLMController
from flask_app.sim_robot_control import move_robot

# Initialize Flask app
app = Flask(__name__)

# Initialize components
llm_controller = LLMController()
bridge = CvBridge()

# Image paths
IMAGE_DIR = 'static/images'
IMAGE_PATHS = {
    'initial': os.path.join(IMAGE_DIR, 'initial.jpg'),
    'current': os.path.join(IMAGE_DIR, 'current.jpg'),
    'previous': os.path.join(IMAGE_DIR, 'previous.jpg'),
    'map': os.path.join(IMAGE_DIR, 'map.jpg')
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
        with frame_lock:
            if latest_frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n\r\n')
        time.sleep(0.1)

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

@app.route('/send_llm_prompt', methods=['POST'])
def send_llm_prompt():
    prompt = request.form.get('prompt')
    if not prompt:
        return render_template('sim_index.html',
                             error="Please enter a prompt")

    try:
        rich.print(f"[blue]Received prompt:[/blue] {prompt}")
        subgoals = llm_controller.generate_subgoals(prompt)

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
    except CvBridgeError as e:
        rich.print(f"[red]Camera error: {e}[/red]")

def load_images():
    """Load all required images"""
    images = {}
    try:
        for img_type, path in IMAGE_PATHS.items():
            if os.path.exists(path):
                with open(path, 'rb') as f:
                    images[img_type] = f.read()
            else:
                rich.print(f"[yellow]Warning: {img_type} image not found[/yellow]")
                return None
        return images
    except Exception as e:
        rich.print(f"[red]Error loading images: {e}[/red]")
        return None

def process_subgoals(prompt, subgoals):
    """Process subgoals for simulation robot"""
    current_subgoal_index = 0
    executed_actions = []
    last_feedback = None

    try:
        # Get initial state
        images = load_images()
        if not images:
            raise Exception("Failed to load initial images")

        rich.print(f"[blue]Processing {len(subgoals)} subgoals for goal:[/blue] {prompt}")

        while current_subgoal_index < len(subgoals):
            current_subgoal = subgoals[current_subgoal_index].split(" ", 1)[1]
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
                last_feedback=last_feedback
            )

            if not validate_control_response(control_response):
                rich.print(f"[red]Invalid control response:[/red] {control_response}")
                continue

            # Save current as previous before executing new action
            with open(IMAGE_PATHS['current'], 'rb') as src:
                with open(IMAGE_PATHS['previous'], 'wb') as dst:
                    dst.write(src.read())

            # Execute action
            if execute_robot_action(control_response):
                executed_actions.append(control_response)
                rich.print(f"[green]Executed action:[/green] {control_response}")
                time.sleep(2)  # Allow time for robot and camera update
            else:
                rich.print("[red]Failed to execute robot action[/red]")
                continue

            # Get fresh images for feedback
            images = load_images()
            if not images:
                continue

            # Get feedback with all 4 images
            feedback = llm_controller.get_feedback(
                initial_image=images['initial'],
                current_image=images['current'],
                previous_image=images['previous'],
                map_image=images['map'],
                current_subgoal=current_subgoal,
                executed_actions=executed_actions,
                last_feedback=last_feedback
            )

            rich.print(f"[purple]Feedback received:[/purple] {feedback}")
            last_feedback = feedback

            # Process feedback
            if feedback == "continue":
                continue
            elif feedback == "subtask complete":
                current_subgoal_index += 1
                executed_actions = []  # Reset for new subtask
                last_feedback = None
            elif feedback == "main goal complete":
                return True
            elif feedback == "no progress":
                last_feedback = "Previous action made no progress, try a different approach"
                continue
            elif feedback.startswith("do"):
                last_feedback = feedback  # Pass suggestion to next control iteration
                continue

        return current_subgoal_index >= len(subgoals)

    except Exception as e:
        rich.print(f"[red]Error in process_subgoals:[/red] {str(e)}")
        return False

# Routes and other functions remain mostly unchanged...

rospy.Subscriber("/xtion/rgb/image_raw", Image, image_callback)

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
