import base64
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from os import getenv, path
from queue import Queue

import rich
from dotenv import load_dotenv
from evaluation_metrics import ExperimentLogger

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
llm_controller = LLMController(simulation=False)
lidar_safety = LidarSafety()
# Thread management
executor = ThreadPoolExecutor(max_workers=3)
command_queue = Queue()
camera_thread = None
llm_thread = None
experiment_logger = ExperimentLogger()


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
        time.sleep(1)

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

@app.route('/stop_robot', methods=['POST', 'GET'])
def stop_robot():
    robot_control.stop_robot()
    # return jsonify({"status": "success", "action": "stop_robot"})
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
                # Get initial state image
        if not fetch_images():
            return render_template('irl_index.html',
                                 error="Failed to fetch initial images",
                                 user_prompt=prompt)

        with open('src/flask_app/static/images/current.jpg', 'rb') as f:
            initial_image = f.read()

        # Generate subgoals with initial image context
        # TODO: add scene description to the generate subgoal model
        session_id = experiment_logger.start_session(prompt, llm_controller.model_name)

        subgoals = llm_controller.generate_subgoals(prompt, initial_image)
        experiment_logger.log_subgoals(subgoals, time.time()- experiment_logger.session_start_time)
        # Generate subgoals
        # subgoals = llm_controller.generate_subgoals(prompt)
        # Log the response
        print(f"Generated subgoals: {subgoals}")

        if not subgoals:
            return render_template('irl_index.html',
                                error="Failed to generate subgoals",
                                user_prompt=prompt)

        #!!process subgoals and control the robot
        success = process_subgoals(prompt, subgoals, robot_control, llm_controller)

        return render_template('irl_index.html',
                             user_prompt=prompt,
                             subgoals=subgoals, success=success)

    except Exception as e:
        print(f"Error in send_llm_prompt: {str(e)}")
        return render_template('irl_index.html',
                             error=str(e),
                             user_prompt=prompt)


def process_subgoals(prompt, subgoals, robot_control, llm_controller):
    """Process subgoals for both simulation and real robot"""
    current_subgoal_index = 0
    executed_actions = []
    last_feedback = None
    initial_image = 'src/flask_app/static/images/initial.jpg'

    global stop_llm
    stop_llm = False

    #

    try:
        # Get initial state image
        if not fetch_images():
            raise Exception("Failed to fetch initial images")

         # Save initial state for the entire task
        with open('src/flask_app/static/images/current.jpg', 'rb') as src:
            with open(initial_image, 'wb') as dst:
                dst.write(src.read())

        rich.print(f"[blue]Processing {len(subgoals)} subgoals for goal:[/blue] {prompt}")

        while current_subgoal_index < len(subgoals):
            try:
                current_subgoal = subgoals[current_subgoal_index].split(" ", 1)[1]  # Remove numbering

                if stop_llm:
                    experiment_logger.complete_session(False, time.time()- experiment_logger.session_start_time)
                    return False
                rich.print(f"\n[cyan]Current subgoal ({current_subgoal_index + 1}/{len(subgoals)}):[/cyan] {current_subgoal}")

                # Get current state images
                if not fetch_images():
                    continue

                try:

                    images = {
                        'initial': initial_image,
                        'current': None,
                        'previous': None,
                        'map': None
                    }
                    with open('src/flask_app/static/images/current.jpg', 'rb') as f:
                        images['current'] = f.read()
                    with open('src/flask_app/static/images/previous.jpg', 'rb') as f:
                        images['previous'] = f.read()
                    with open('src/flask_app/static/images/map.jpg', 'rb') as f:
                        images['map'] = f.read()
                except Exception as e:
                    rich.print(f"[red]Error loading images: {e}[/red]")
                    continue

                # Get control response
                safety_context = lidar_safety.get_safety_context()

                safety_context = lidar_safety.get_safety_context()

                control_response = llm_controller.control_robot(
                    subgoal=current_subgoal,
                    initial_image=images['initial'],
                    current_image=images['current'],
                    previous_image=images['previous'],
                    map_image=images['map'],
                    executed_actions=executed_actions,
                    last_feedback=last_feedback,
                    all_subgoals = subgoals,
                    safety_warning=None,
                    safety_context=safety_context
                )

                # handle invalid control response
                if not validate_control_response(control_response):
                    rich.print(f"[red]Invalid control response:[/red] {control_response}")
                    experiment_logger.log_invalid_control(current_subgoal, control_response)
                    last_feedback = f"{control_response} is an Invalid action to generate"
                    continue

                # Check safety before execution
                is_safe, warning = lidar_safety.check_direction_safety(control_response)


                if not is_safe:
                    last_feedback = f"Safety warning: {warning}"
                    experiment_logger.log_safety_trigger(current_subgoal, warning)

                    # Update safety context
                    safety_context = lidar_safety.get_safety_context()

                    # Get new Control with safty warning
                    control_response = llm_controller.control_robot(
                                    subgoal=current_subgoal,
                                    initial_image=images['initial'],
                                    current_image=images['current'],
                                    previous_image=images['previous'],
                                    map_image=images['map'],
                                    executed_actions=executed_actions,
                                    last_feedback=last_feedback,
                                    all_subgoals=subgoals,
                                    safety_warning=warning,
                                    safety_context=safety_context
                                )

                    if not validate_control_response(control_response):
                        rich.print(f"[red]Invalid control response:[/red] {control_response}")
                        experiment_logger.log_invalid_control(current_subgoal, control_response)
                        last_feedback = f"{control_response} is an Invalid action to generate"
                        continue

                    # Log recovery attempt
                    experiment_logger.log_safety_recovery(current_subgoal, control_response)

                    # Update safety context
                    safety_context = lidar_safety.get_safety_context()

                    # Get new Control with safty warning
                    control_response = llm_controller.control_robot(
                                    subgoal=current_subgoal,
                                    initial_image=images['initial'],
                                    current_image=images['current'],
                                    previous_image=images['previous'],
                                    map_image=images['map'],
                                    executed_actions=executed_actions,
                                    last_feedback=last_feedback,
                                    all_subgoals=subgoals,
                                    safety_warning=warning,
                                    safety_context=safety_context
                                )

                    if not validate_control_response(control_response):
                        rich.print(f"[red]Invalid control response:[/red] {control_response}")
                        experiment_logger.log_invalid_control(current_subgoal, control_response)
                        last_feedback = f"{control_response} is an Invalid action to generate"
                        continue

                    # Log recovery attempt
                    experiment_logger.log_safety_recovery(current_subgoal, control_response)

                    continue


                # Save current as previous before executing action
                with open('src/flask_app/static/images/current.jpg', 'rb') as src:
                    with open('src/flask_app/static/images/previous.jpg', 'wb') as dst:
                        dst.write(src.read())


                success = execute_robot_action(control_response, current_subgoal)
                if success:
                    # evaluator.log_action(control_response)
                    executed_actions.append(control_response)
                    rich.print(f"[green]Executed action:[/green] {control_response}")
                    time.sleep(0.1)  # Allow time for action completion
                else:
                    last_feedback = f"Failed to execute robot action: CHOOSE A DIFFERENT ACTION."
                    experiment_logger.log_invalid_control(current_subgoal, control_response)
                    rich.print(f"[red]Failed to execute robot action: Safety issue[/red]")
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

                feedback = llm_controller.get_feedback(
                    initial_image=images['initial'],
                    current_image=images['current'],
                    previous_image=images['previous'],
                    map_image=images['map'],
                    current_subgoal=current_subgoal,
                    executed_actions=executed_actions,
                    last_feedback=last_feedback,
                    subgoals = subgoals
                )

                rich.print(f"[purple]Feedback received:[/purple] {feedback}")
                experiment_logger.log_feedback(current_subgoal, feedback)
                last_feedback = feedback

                # Process feedback
                if feedback.startswith("adjust:"):
                    adjustment = feedback.split(":", 1)[1]
                    last_feedback = f"Previous approach ineffective: {adjustment}"
                    continue
                elif feedback == "continue" or "continue" in feedback:
                    continue
                elif feedback == "subtask complete" or "subtask complete" in feedback:
                    current_subgoal_index += 1
                    executed_actions = []  # Reset for new subtask
                    last_feedback = None
                    experiment_logger.log_feedback(current_subgoal, "subtask complete")
                elif feedback == "main goal complete" or "main goal complete" in feedback:
                    return True
                elif feedback == "no progress" or "no progress" in feedback:
                    last_feedback = f"previous action '{control_response}' has not gotten the robot closer to completing the task yet"
                    continue
                # elif feedback.startswith("do") or feedback.startswith("based"):
                #     last_feedback = feedback  # Pass suggestion to next control iteration
                #     continue
                else:
                    last_feedback = feedback
                    continue


            except Exception as e:
                rich.print(f"[red]Error  in control flow of processing subgoal:[/red] {str(e)}")
                last_feedback = f"Error processing subgoal: {str(e)}"
                continue
        # return current_subgoal_index >= len(subgoals)
         # Task completed through all subgoals
        success = current_subgoal_index >= len(subgoals)
        experiment_logger.complete_session(True, time.time()- experiment_logger.session_start_time)
        return success

    except Exception as e:
        rich.print(f"[red]Error in main part of process_subgoals:[/red] {str(e)}")
        return False

def validate_control_response(response):

    """Validate that control response is one of allowed actions"""
    valid_actions = [
        "move forward",
        "move backward",
        "turn left",
        "turn right",
        "completed",
    ]

    basic_actions = {
        "move forward": ("move_forward", None),
        "move backward": ("move_backward", None),
        "turn left": ("turn_left", None),
        "turn right": ("turn_right", None),
        "completed": ("completed", None)
    }

    basic_actions = {
        "move forward": ("move_forward", None),
        "move backward": ("move_backward", None),
        "turn left": ("turn_left", None),
        "turn right": ("turn_right", None),
        "completed": ("completed", None)
    }
    # check whole response if it is in the valid actions
    # return response in valid_actions
    # return response and response.lower() in valid_actions
    # Check basic commands first
    response = response.lower().strip()
    if response in basic_actions:
        return True, basic_actions[response]

    # Parse parameterized commands
    try:
        parts = response.split()
        if len(parts) >= 7:  # e.g. "move forward 0.5 meters at 0.3 m/s"
            action = " ".join(parts[0:2])
            value = float(parts[2])
            speed = float(parts[5].rstrip("m/s"))

            if action == "move forward" and "meters" in response:
                if 0.1 <= value <= 2.0 and 0.1 <= speed <= 0.6:
                    return True, ("move_forward_by", (value, speed))
                elif value > 2.0 or speed > 0.5:
                    return True , ("move_forward_by", (2.0, 0.6))

            elif (action in ["turn left", "turn right"]) and "degrees" in response:
                if 1 <= value <= 180 and 0.1 <= speed <= 0.6:
                    return True, ("turn_by_angle", (value * (1 if action == "turn left" else -1), speed))
                elif value > 180 or speed > 0.5:
                    return True, ("turn_by_angle", (180 * (1 if action == "turn left" else -1), 0.6))

    except (ValueError, IndexError):
        print(f"Validation error: Invalid command format")

    return False, None

# ACTION_TIMEOUT = 10  # seconds
def execute_robot_action(action, current_subgoal):
    """Execute robot action based on the response"""
    try:
        response = action.lower().strip()

        # Handle basic commands
        if response == "move forward":
            experiment_logger.log_action(current_subgoal, "move forward")
            return robot_control.move_forward()
        elif response == "turn left":
            experiment_logger.log_action(current_subgoal, "move left")

            return robot_control.turn_left()
        elif response == "turn right":
            experiment_logger.log_action(current_subgoal, "move right")

            return robot_control.turn_right()
        elif response == "completed":
            experiment_logger.log_action(current_subgoal, "completed")

            return True

        else:
            # Handle parameterized commands
            parts = response.split()
            if len(parts) >= 4:
                action = " ".join(parts[0:2])
                value = float(parts[2])
                speed = float(parts[5].rstrip("m/s"))


                if action == "move forward" and "meters" in response:
                    experiment_logger.log_action(current_subgoal, response)

                    return robot_control.move_forward_by(value, speed)
                elif action == "turn left" and "degrees" in response:
                    experiment_logger.log_action(current_subgoal, response)

                    return robot_control.turn_by_angle(value, speed)
                elif action == "turn right" and "degrees" in response:
                    experiment_logger.log_action(current_subgoal, response)

                    return robot_control.turn_by_angle(-value, speed)
                else:
                    return False
        # return False, "Invalid command format"

    except Exception as e:
        return False, f"Action failed: {str(e)}"
    # try:
    #     is_valid, command_data = validate_control_response(action)
    #     if not is_valid:
    #         return False, "Invalid command format"

    #     command, params = command_data
    #     start_time = time.time()

    #     while time.time() - start_time < ACTION_TIMEOUT:
    #         is_safe, warning = lidar_safety.check_direction_safety(command)
    #         if not is_safe:
    #             return False, f"Safety warning: {warning}"

    #         if params is None:  # Basic commands
    #             if command == "move_forward":
    #                 return robot_control.move_forward()
    #             elif command == "turn_left":
    #                 return robot_control.turn_left()
    #             elif command == "turn_right":
    #                 return robot_control.turn_right()
    #             elif command == "completed":
    #                 return True, "Task completed"
    #         else:  # Parameterized commands
    #             if command == "move_forward_by":
    #                 distance, speed = params
    #                 return robot_control.move_forward_by(distance, speed)
    #             elif command == "turn_by_angle":
    #                 angle, speed = params
    #                 return robot_control.turn_by_angle(angle, speed)
    #     return False, "Action timed out"
    # except Exception as e:
    #     return False, f"Action failed: {str(e)}"

    #     start_time = time.time()
    #     while time.time() - start_time < ACTION_TIMEOUT:
    #         is_safe, warning = lidar_safety.check_direction_safety(action)

    #         if not is_safe:
    #             rich.print(f"[red]Safety check failed:[/red] {warning}")
    #             return False, f"Safety warning: {warning}"

    #         if action == "move forward":
    #             return robot_control.move_forward()
    #             # return True

    #         elif action == "move backward":
    #             return robot_control.move_backward()
    #             # return True

    #         elif action == "turn left":
    #             return robot_control.turn_left()

    #         elif action == "turn right":
    #             return robot_control.turn_right()
    #         elif action == "completed":
    #             return True
    #         else:
    #             print(f"Invalid action: {action}")
    #             return False , "Invalid action"
    #     return False, "Action timed out"
    # except Exception as e:
    #     return False, f"Action failed: {str(e)}"
    # try:
    #     is_valid, command_data = validate_control_response(action)
    #     if not is_valid:
    #         return False, "Invalid command format"

    #     command, params = command_data
    #     start_time = time.time()

    #     while time.time() - start_time < ACTION_TIMEOUT:
    #         is_safe, warning = lidar_safety.check_direction_safety(command)
    #         if not is_safe:
    #             return False, f"Safety warning: {warning}"

    #         if params is None:  # Basic commands
    #             if command == "move_forward":
    #                 return robot_control.move_forward()
    #             elif command == "turn_left":
    #                 return robot_control.turn_left()
    #             elif command == "turn_right":
    #                 return robot_control.turn_right()
    #             elif command == "completed":
    #                 return True, "Task completed"
    #         else:  # Parameterized commands
    #             if command == "move_forward_by":
    #                 distance, speed = params
    #                 return robot_control.move_forward_by(distance, speed)
    #             elif command == "turn_by_angle":
    #                 angle, speed = params
    #                 return robot_control.turn_by_angle(angle, speed)
    #     return False, "Action timed out"
    # except Exception as e:
    #     return False, f"Action failed: {str(e)}"

    #     start_time = time.time()
    #     while time.time() - start_time < ACTION_TIMEOUT:
    #         is_safe, warning = lidar_safety.check_direction_safety(action)

    #         if not is_safe:
    #             rich.print(f"[red]Safety check failed:[/red] {warning}")
    #             return False, f"Safety warning: {warning}"

    #         if action == "move forward":
    #             return robot_control.move_forward()
    #             # return True

    #         elif action == "move backward":
    #             return robot_control.move_backward()
    #             # return True

    #         elif action == "turn left":
    #             return robot_control.turn_left()

    #         elif action == "turn right":
    #             return robot_control.turn_right()
    #         elif action == "completed":
    #             return True
    #         else:
    #             print(f"Invalid action: {action}")
    #             return False , "Invalid action"
    #     return False, "Action timed out"
    # except Exception as e:
    #     return False, f"Action failed: {str(e)}"
# endregion

stop_llm = False

@app.route('/stop_llm', methods=['POST'])
def stop_llm_control():
    """Stop LLM control and generate evaluation report"""
    # Generate report if task in progress
    global stop_llm
    try:
        if experiment_logger.current_session:
            session_duration = time.time() - experiment_logger.session_start_time
            stop_llm = True
            experiment_logger.complete_session(False, session_duration)
            rich.print("[yellow]LLM control stopped, evaluation saved[/yellow]")
            rich.print(f"[blue]Session duration: {session_duration:.2f}s[/blue]")

        return redirect(url_for('index'))

    except Exception as e:
        rich.print(f"[red]Error stopping LLM: {str(e)}[/red]")
        # Log the error for debugging
        return redirect(url_for('index'))

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001, debug=True)