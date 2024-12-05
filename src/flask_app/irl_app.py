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
from lidar_safety import LidarSafety

# from LLM_robot_control_models import control_robot, generate_subgoals, get_feedback
from LLM_robot_control_models import LLMController

# from mapping import OccupancyMapper
from PIL import Image
from robot_control_over_ssh import RobotControl
from sensor_data import fetch_images, trigger_capture_script, trigger_stop_script

app = Flask(__name__)
robot_control = RobotControl()
llm_controller = LLMController()
lidar_safety = LidarSafety()
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
# def process_subgoals(prompt, subgoals):
#     """Process the subgoals and send to robot"""
#     try:
#         current_subgoal_index = 0
#         executed_actions = []
#         last_feedback = None

#         print(f"Processing {len(subgoals)} subgoals")
#         print(f"Main goal: {prompt}")
#         while current_subgoal_index < len(subgoals):
#             current_subgoal = subgoals[current_subgoal_index]
#             print(f"\nProcessing subgoal {current_subgoal_index + 1}: {current_subgoal}")
#             print(f"Executed actions: {executed_actions}")
#             print(f"Last feedback: {last_feedback}")

#             if not fetch_images():
#                 # raise Exception("Failed to fetch required images")
#                 print("Failed to fetch required images")
#                 time.sleep(1)
#                 continue
#             try:
#                 # Get current images
#                 with open('src/flask_app/static/images/current.jpg', 'rb') as f:
#                     current_image = f.read()
#                 with open('src/flask_app/static/images/map.jpg', 'rb') as f:
#                     map_image = f.read()
#             except Exception as e:
#                 print(f"Error reading images: {e}")
#                 continue


#             current_subgoal = subgoals[current_subgoal_index]
#             # print(f"Control response: {control_response}")
#             rich.print(f"\n [blue]Processing Current subgoal[/blue]: {current_subgoal}")
#             print(f"Previous actions: {executed_actions}")
#             print(f"Last feedback: {last_feedback}")
#              # Get control action
#             #remove numbering from subgoal
#             # current_subgoal = current_subgoal.split(" ", 1)[1]
#             control_response = llm_controller.control_robot(

#                 current_subgoal.split(" ", 1)[1],
#                 current_image,
#                 map_image,
#                 executed_actions,
#                 last_feedback)

#             rich.print(f"[yellow]Feedback response:[yellow] {control_response}")

#             if validate_control_response(control_response):
#                 # Save image before action
#                 try:
#                 # with open('src/flask_app/static/images/previous.jpg', 'wb') as f:
#                 #     previous_image = f.write(current_image)
#                     with open('src/flask_app/static/images/current.jpg', 'rb') as src:
#                         with open('src/flask_app/static/images/previous.jpg', 'wb') as dst:
#                             dst.write(src.read())
#                 except Exception as e:
#                         print(f"Error saving previous image: {e}")
#                         continue
#                     # f.write(current_image)

#                  # Execute robot action
#                 if execute_robot_action(control_response):
#                      executed_actions.append(control_response)
#                      rich.print(f"[blue] Updated executed actions:[/blue] {executed_actions}")
#                      time.sleep(2)
#                 else:
#                     rich.print("[red]Failed to execute robot action[/red]")
#                     # control_response = llm_controller.control_robot(
#                     #     current_subgoal,
#                     #     current_image,
#                     #     map_image,
#                     #     executed_actions,
#                     #     "failed to execute the last action try again")

#                     # validate_control_response(control_response)
#                     continue

#                 # time.sleep(2)  # Wait for robot to complete action

#                 # # Execute robot action
#                 # execute_robot_action(control_response)
#                 # time.sleep(1)


#                 # Get new image after action
#                  # Read new current image for feedback
#                 try:
#                     # Fetch fresh images again after action
#                     if not fetch_images():
#                         rich.print("[red] Failed to fetch post-action images[/red]")
#                         # continue
#                     with open('src/flask_app/static/images/current.jpg', 'rb') as f:
#                         new_current_image = f.read()
#                     with open('src/flask_app/static/images/previous.jpg', 'rb') as f:
#                         previous_image = f.read()
#                 except Exception as e:
#                     print(f"Error reading feedback images: {e}")
#                     continue
#                 # with open('src/flask_app/static/images/current.jpg', 'rb') as f:
#                 #     current_image = f.read()

#                 # Get feedback
#                 feedback = llm_controller.get_feedback(
#                     new_current_image,
#                     previous_image,
#                     current_subgoal,
#                     executed_actions,
#                     last_feedback)
#                 last_feedback = feedback
#                 rich.print(f"[purple]Feedback recieved:[/purple] {feedback}")
#                 print(f"updated feedback context: {last_feedback}")

#                 if feedback == "continue":
#                     print("Progress made, continuing to with current subtask")
#                     continue
#                 elif feedback == "subtask complete":
#                     print(f"completed Subtask: {current_subgoal_index + 1},\n moving to next subtask")
#                     current_subgoal_index += 1
#                     executed_actions = [] # reset executed actions
#                     last_feedback = None
#                 elif feedback == "no progress":
#                     print("No progress made, retrying action")

#                     control_response = llm_controller.control_robot(
#                         current_subgoal,
#                         current_image,
#                         map_image,
#                         executed_actions,
#                         "no significant progress hasnt been made to completing the task based on your previous actions")
#                     continue
#                 elif feedback.startswith("do") or feedback.startswith("based"):
#                     # feedback to try recommending a different action

#                     control_response = llm_controller.control_robot(
#                         current_subgoal,
#                         current_image,
#                         map_image,
#                         executed_actions,
#                         feedback)
#                 elif feedback == "main goal complete":
#                     print("Main goal complete")
#                     return True
#                 else:
#                     rich.print(f"[orange3]Invalid control response:[/orange3] {control_response}")
#                     continue

#     except Exception as e:
#         print(f"Error processing subtasks: {e}")
#         return False

#     rich.print("[green_yellow]All subgoals completed[/green_yellow]")
#     return True

# def validate_control_response(response):

#     """Validate that control response is one of allowed actions"""
#     valid_actions = [
#         "move forward",
#         "move backward",
#         "turn left",
#         "turn right",
#     ]
#     # check whole response if it is in the valid actions
#     # return response in valid_actions
#     return response and response.lower() in valid_actions

def process_subgoals(prompt, subgoals, robot_control, llm_controller):
    """Process subgoals for both simulation and real robot"""
    current_subgoal_index = 0
    executed_actions = []
    last_feedback = None
    initial_image = None

    try:
        # Get initial state image
        if not fetch_images():
            raise Exception("Failed to fetch initial images")

         # Save initial state for the entire task
        with open('src/flask_app/static/images/current.jpg', 'rb') as src:
            with open('src/flask_app/static/images/initial.jpg', 'wb') as dst:
                initial_image = src.read()
                dst.write(initial_image)

        rich.print(f"[blue]Processing {len(subgoals)} subgoals for goal:[/blue] {prompt}")

        while current_subgoal_index < len(subgoals):
            current_subgoal = subgoals[current_subgoal_index].split(" ", 1)[1]  # Remove numbering
            rich.print(f"\n[cyan]Current subgoal ({current_subgoal_index + 1}/{len(subgoals)}):[/cyan] {current_subgoal}")

            # Get current state images
            if not fetch_images():
                continue

            # Load all required images
            images = {
                'initial': initial_image,
                'current': None,
                'previous': None,
                'map': None
            }

            try:
                with open('src/flask_app/static/images/current.jpg', 'rb') as f:
                    images['current'] = f.read()
                with open('src/flask_app/static/images/previous.jpg', 'rb') as f:
                    images['previous'] = f.read()
                with open('src/flask_app/static/images/map.jpg', 'rb') as f:
                    images['map'] = f.read()
            except Exception as e:
                rich.print(f"[red]Error loading images: {e}[/red]")
                continue


            # control loop with context
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

            # Save current as previous before executing action
            with open('src/flask_app/static/images/current.jpg', 'rb') as src:
                with open('src/flask_app/static/images/previous.jpg', 'wb') as dst:
                    dst.write(src.read())

            # Execute action and update history
            # if execute_robot_action(control_response, robot_control):
            #     executed_actions.append(control_response)
            #     rich.print(f"[green]Executed action:[/green] {control_response}")
            #     time.sleep(2)  # Allow time for action completion
            # else:
            #     rich.print("[red]Failed to execute robot action[/red]")
            #     continue
            success, safety_warning = execute_robot_action(control_response)
            if success:
                executed_actions.append(control_response)
                rich.print(f"[green]Executed action:[/green] {control_response}")
                time.sleep(2)  # Allow time for action completion
            else:
                if safety_warning:
                    last_feedback = f"Safety warning: {safety_warning}. Choose a different action."
                rich.print(f"[red]Failed to execute robot action: {safety_warning}[/red]")
                continue

            # Get feedback with updated images
            if not fetch_images():
                continue

            try:
                with open('src/flask_app/static/images/current.jpg', 'rb') as f:
                    new_current_image = f.read()
                with open('src/flask_app/static/images/previous.jpg', 'rb') as f:
                    new_previous_image = f.read()
            except Exception as e:
                rich.print(f"[red]Error reading feedback images: {e}[/red]")
                continue

            # feedback = llm_controller.get_feedback(
            #     initial_image=initial_image,
            #     current_image=new_current_image,
            #     previous_image=new_previous_image,
            #     map_image=map_image,
            #     current_subgoal=current_subgoal,
            #     executed_actions=executed_actions,
            #     last_feedback=last_feedback
            # )
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
            elif feedback.startswith("do") or feedback.startswith("based"):
                last_feedback = feedback  # Pass suggestion to next control iteration
                continue

        return current_subgoal_index >= len(subgoals)

    except Exception as e:
        rich.print(f"[red]Error in process_subgoals:[/red] {str(e)}")
        return False

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
    return response and response.lower() in valid_actions

ACTION_TIMEOUT = 10  # seconds
def execute_robot_action(action):
    """Execute robot action based on the response"""
    try:
        start_time = time.time()
        while time.time() - start_time < ACTION_TIMEOUT:
            is_safe, warning = lidar_safety.check_direction_safety(action)

            if not is_safe:
                rich.print(f"[red]Safety check failed:[/red] {warning}")
                return False, f"Safety warning: {warning}"

            if action == "move forward":
                return robot_control.move_forward()
                # return True

            elif action == "move backward":
                return robot_control.move_backward()
                # return True

            elif action == "turn left":
                return robot_control.turn_left()

            elif action == "turn right":
                return robot_control.turn_right()
            else:
                print(f"Invalid action: {action}")
                return False
        return False, "Action timed out"
    except Exception as e:
        return False, f"Action failed: {str(e)}"

#TODO: check lidar if there is an obstacle and the next commad is to move {direction of obstacle} reprompt the llm telling it that there is an obstacle
# endregion

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001, debug=True)