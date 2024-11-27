# region Imports
import base64
import os
import string
import threading

import cv2
import google.api_core.exceptions
import ollama
import rich
import rospy  # type: ignore #* ROS Python client library
import vertexai
from cv_bridge import (  # type: ignore #* convert ROS messages to OpenCV images
    CvBridge,
    CvBridgeError,
)
from flask import Flask, Response, jsonify, redirect, render_template, request, url_for
from gemini_config import (
    generation_config,
    goal_setter_system_prompt,
    safety_settings,
    system_prompt,
)
from robot_control import (
    VALID_DIRECTIONS,
    control_gripper,
    execute_with_feedback,
    extend_arm,
    move_head,
    move_robot,
    retract_arm,
    rotate_arm,
    set_pre_pick,
    set_tucked_in,
    update_arm,
    update_torso,
)
from safe import CREDENTIALS, PROJECT_ID, REGIONNA
from sensor_msgs.msg import (  # type: ignore #* camera and lidar data
    Image,
    JointState,
    LaserScan,
)
from vertexai.preview.generative_models import GenerativeModel
from vertexai.preview.generative_models import Image as GeminiImage

# endregion Imports

# region Flask and ROS config

os.environ["GOOGLE_APPLICATIN_CREDENTIALS"] = CREDENTIALS
app = Flask(__name__)

bridge = CvBridge()

#* Global variables to store latest camera frame and lidar data
latest_frame = None
lidar_data = None
current_arm_position = None
gemini_response:str = None
stop_gemini:bool = False
frame_lock:threading.Lock = threading.Lock() # protect process from being accessed by multiple threads at the same time
lidar_lock:threading.Lock = threading.Lock()
arm_position_lock:threading.Lock = threading.Lock()
gemini_response_lock:threading.Lock = threading.Lock()
VALID_INSTRUCTIONS = ["move forward", "move backward", "move left", "move right", "turn left", "turn right", "stop", "arm pre_grab", "arm tucked_in", "arm reach_forward", "arm retract", "arm rotate", "head left", "head right", "head up", "head down", "control_gripper open", "control_gripper close"]


# endregion

# region Home
# #* Home
@app.route('/')
def index():
    return render_template('index.html', response="", arm_positions=ARM_POSITIONS, current_position=current_position)

# endregion

# region Gemini

# region Gemini_init_varaibles
vertexai.init(project=PROJECT_ID, location=REGIONNA)
MAX_REQUESTS_PER_MINUTE = 4
WAIT_TIME = 10

gemini_response_history = []
llava_response = None
subgoals = []
current_subgoal_index = 0
# endregion Gemini_init_varaibles

# region get_camera_image
def get_camera_image():
    global latest_frame
    with frame_lock:
        if latest_frame is not None:
            _, buffer = cv2.imencode('.jpg', latest_frame)
            image_str = buffer.tobytes()
            return image_str
        # new_positions[2] += 0.1

# region models
#todo: use llm_robot_control_models.py for simulation
goal_setter = GenerativeModel("gemini-1.0-pro", system_instruction=[goal_setter_system_prompt])
# control_model = GenerativeModel("gemini-1.5-flash", system_instruction=[system_prompt])
# verifier =
# endregion models

# region multiturn_generate_content
def multiturn_generate_content(model, message=[], generation_config=None, safety_settings=None):
    generative_multimodal_model = model
    try:
        chat = generative_multimodal_model.start_chat()
        # api_response = chat.send_message([image, message], generation_config=generation_config, safety_settings=safety_settings)
        api_response = chat.send_message(message, generation_config=generation_config, safety_settings=safety_settings)
        return api_response
    except (google.api_core.exceptions.ResourceExhausted,vertexai.generative_models._generative_models.ResponseValidationError) as e :
        print(f"Error in multiturn_generate_content: {e}")
        return None
# endregion multiturn_generate_content

# region generate_subgoals
def generate_subgoals(prompt:str) -> str:
    """
    gemini model takes user input and generates a set of subgoals to cahive the given prompt
    """
    try:
        goal_setter_response = multiturn_generate_content(goal_setter, message=prompt, generation_config=generation_config, safety_settings=safety_settings)
        if goal_setter_response and goal_setter_response.candidates:
            _subgoals = goal_setter_response.candidates[0].text.split('\n')
            rich.print(f"[green]Gemini[green]: {_subgoals}")
            return _subgoals
    except google.api_core.exceptions.ResourceExhausted:
        print("ResourceExhausted in generating subgoals")
        # return None
    except vertexai.generative_models._generative_models.ResponseValidationError:
        print("ResponseValidationError in generating subgoals")
    return None

# endregion generate_subgoals

# region llava_control
def llava_control(prompt, image_str, llava_system_prompt=system_prompt ):
    """ take gemini response (subgoals) and generate executable action(s) using llava
    """
    print("running llava_control")
    image_base64 = base64.b64encode(image_str).decode('utf-8')

    # Combine the prompt and the image into a single multimodal message
    message = [
        {"role": "system", "content": llava_system_prompt},
        {"role": "user", "content": image_base64, "is_image": True},
        {"role": "user", "content": prompt},
    ]
    # print(message)
    try:
        response = ollama.chat(
            model='llava-llama3',
            messages=message,
            stream=False,
            options={'temperature': 0.2, 'top_p': 0.98,'num_predict':4,'max_tokens': 20,
                'safety_settings': {
                    'use_safety_model': True,
                    'safety_model': 'openai/safetensors',
                    'safety_threshold': 0.5,
                    'safety_top_p': 0.95,
                    'safety_temperature': 0.7
                }}
        )
        return response['message']['content']
    except Exception as e:
        rich.print(f"[red]Error in llava_generate[red]: {e}")
        return None


# endregion llava_control

# region llava_control_loop

def llava_control_loop(prompt, subgoals):
    global llava_response, stop_gemini, gemini_response_history, current_subgoal_index
    stop_gemini = False
    executed_actions=[]

    while not stop_gemini and current_subgoal_index < len(subgoals):
        image_str = get_camera_image()
        if image_str is None:
            print("No image")
            continue

        current_subgoal = subgoals[current_subgoal_index]

        actions_summary = "\n".join(executed_actions)

        llava_prompt = f"""the goal is: {prompt}.
        \nyou can do this by {subgoals}
        \nyour current task is to {current_subgoal}
        Actions executed: {actions_summary if actions_summary != "" else "None"}

        reply with 'done!!' to move to the next text; use the image as a guide"""
        # llava_control_response = llava_control(f"the goal is: {prompt}.\nyou can do this by {subgoals}\nyour current task is to {current_subgoal} reply with 'done!!' to move to the next text; use the image as a guide", image_str)

        llava_control_response = llava_control(llava_prompt, image_str)



        if not llava_control_response:
            continue

        feedback = process_llava_response(llava_control_response, current_subgoal, image_str)
        if feedback == "invalid instruction":
            llava_control_response = llava_control(f"The response you gave '{llava_response}' is not valid. try again\n{llava_prompt}", image_str)
            feedback = process_llava_response(llava_control_response, current_subgoal, image_str)
        else:
            update_gemini_history(current_subgoal, llava_response, feedback)
        # previous_responses = llava_response

        if "done" in llava_response.lower():
            current_subgoal_index += 1
            if current_subgoal_index >= len(subgoals):
                stop_gemini = True
                print("All subgoals completed")
            else:
                print("Moving to next subgoal")

        rospy.sleep(3)

def process_llava_response(response, current_subgoal, image_str):
    global llava_response
    llava_response = response.strip().lower()

    if not any(llava_response.startswith(instruction) for instruction in VALID_INSTRUCTIONS):
        return "invalid instruction"

    if llava_response == "failed to understand":
        return "LLaVA failed to understand. Retrying..."

    if llava_response == "done!!":
        return "Subgoal completed."

    parts = llava_response.split()
    command = parts[0]
    params = parts[1:] if len(parts) > 1 else []

    if command == "move":
        return execute_with_feedback(move_robot, params[0])
    elif command == "arm":
        return execute_arm_command(params)
    elif command == "head":
        return execute_with_feedback(move_head, params[0])
    elif command == "control_gripper":
        return execute_with_feedback(control_gripper, params[0])
    else:
        return "Command recognized but not implemented."

def execute_arm_command(params):
    if params[0] == "pre_grab":
        return execute_with_feedback(set_pre_pick)
    elif params[0] == "tucked_in":
        return execute_with_feedback(set_tucked_in)
    elif params[0] == "reach_forward":
        return execute_with_feedback(extend_arm, float(params[1]))
    elif params[0] == "retract":
        return execute_with_feedback(retract_arm, float(params[1]))
    elif params[0] == "rotate":
        return execute_with_feedback(rotate_arm, params[1])
    else:
        return "Invalid arm command."

def update_gemini_history(current_subgoal, llava_response, feedback):
    with gemini_response_lock:
        gemini_response_history.append(f"subgoal: {current_subgoal} \n {llava_response}")
        gemini_response_history.append(f"Feedback: {feedback}")

# endregion llava_control_loop


# region Gemini_flask
@app.route('/send_prompt', methods=['POST'])
def send_prompt():
    global gemini_response, subgoals, current_subgoal_index
    user_prompt = request.form.get('prompt')

    gemini_response_history.clear() # clearing history
    gemini_response_history.append(f"user: {user_prompt}")

    subgoals = generate_subgoals(user_prompt)
    if subgoals:
        gemini_response_history.append(f"Gemini (subgoals): {subgoals}")

    current_subgoal_index = 0 # reset index

    # asyncio.create_task(llava_control_loop(user_prompt, subgoals))
    # await llava_control_loop(user_prompt, subgoals)
    llava_thread = threading.Thread(target=llava_control_loop, args=(user_prompt, subgoals))
    llava_thread.start()

    return render_template('index.html',
                           goal_setter_response=subgoals,
                           control_model_response=gemini_response,
                           arm_positions=ARM_POSITIONS.keys(),
                           current_position=get_current_arm_position_name(current_arm_position))

@app.route('/get_responses')
def get_responses():
    global gemini_response_history
    return jsonify({'responses': gemini_response_history})

@app.route('/stop_gemini', methods=['POST'])
def stop_gemini_control():
    global stop_gemini
    stop_gemini = True
    print("gemini stop")
    return redirect(url_for('index'))
# endregion Gemini_flask
# endregion Gemini

# region Sensors
# region camera
#TODO: save current.jpg to a file and save previous.jpg for the llm prompt
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
        rospy.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# endregion camera

# region lidar
#TODO: create map.jpg from lidar data(occupancy grid) for the llm prrompt
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