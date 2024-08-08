# region Imports
import base64
import string
from flask import Flask, jsonify, request, Response, render_template, redirect, url_for
import rospy # type: ignore #* ROS Python client library
from sensor_msgs.msg import Image, LaserScan, JointState# type: ignore #* camera and lidar data
import cv2
from cv_bridge import CvBridge, CvBridgeError # type: ignore #* convert ROS messages to OpenCV images
import threading
import vertexai
from vertexai.preview.generative_models import GenerativeModel
from vertexai.preview.generative_models import Image as GeminiImage
from safe import PROJECT_ID, REGIONNA, CREDENTIALS
from gemini_config import generation_config, safety_settings, system_prompt, goal_setter_system_prompt
import os
from robot_control import VALID_DIRECTIONS, control_gripper, execute_with_feedback, extend_arm, move_head, move_robot, retract_arm, rotate_arm, set_pre_pick, set_tucked_in, update_torso, update_arm
import google.api_core.exceptions
import ollama
import asyncio
# endregion Imports

# region Flask and ROS config
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = CREDENTIALS
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
    return None
# endregion get_camera_image

# region models
goal_setter = GenerativeModel("gemini-1.0-pro", system_instruction=[goal_setter_system_prompt])
control_model = GenerativeModel("gemini-1.5-flash", system_instruction=[system_prompt])
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
            return _subgoals
    except google.api_core.exceptions.ResourceExhausted:
        print("ResourceExhausted in generating subgoals")
        # return None
    except vertexai.generative_models._generative_models.ResponseValidationError:
        print("ResponseValidationError in generating subgoals")
    return None

# endregion generate_subgoals

# region llava_control
def llava_control(prompt,image_str, llava_system_prompt=system_prompt ):
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
        print(f"llava response: {response['message']}")
        return response['message']['content']
    except Exception as e:
        print(f"Error in llava_generate: {e}")
        return None


# endregion llava_control

# region llava_control_loop
def llava_control_loop(prompt, subgoals):
    print("hello llava")
    print(subgoals)
    print(len(subgoals))
    global llava_response, stop_gemini, request_count,  gemini_response_history, current_subgoal_index
    stop_gemini = False



    while not stop_gemini and current_subgoal_index < len(subgoals):
        image_str = get_camera_image()
        if image_str is None:
            print("No image")
            continue

        current_subgoal = subgoals[current_subgoal_index]
        llava_control_response = llava_control(f"the goal is: {prompt} and your current task is to {current_subgoal}",image_str )

        if llava_control_response:
            with gemini_response_lock:
                llava_response = llava_control_response.strip()
                # remove all punctuation marks
                llava_response = llava_response.translate(str.maketrans('', '', string.punctuation))

                #remove space from the start of the response
                llava_response = llava_response.lstrip().lower()
                # remove number at start of response if ther is one
                llava_response = llava_response.split(" ", 1)[1]
                print(f"llava_response: {llava_response}")


                # if lava response is longer than two words and isnt failed to understand prompt reprompt the llm
                if len(llava_response.split(' ')) > 4 and "failed to understand prompt" not in llava_response:
                    print("invalid llava_response: ", llava_response)
                    llava_control_response = llava_control(f" the response '{llava_response}' is not in the list of 13 valid robot instructions, try again the goal is: {prompt} and your current task is to {current_subgoal} the image is provided",image_str )
                else:
                    print(f"llava_response: {llava_response}")
                gemini_response_history.append(f"subgoal: {current_subgoal} \n {llava_response}")

                feedback = ""
                if llava_response.startswith("move"):
                    direction = llava_response.split(" ")[1]
                    if direction in VALID_DIRECTIONS:
                        feedback = execute_with_feedback(move_robot, direction)
                    else:
                        feedback = "Invalid direction. Please try again."
                        llava_control_response = llava_control(f" '{llava_response}' is not a valid robot instruction listed, try again the goal is: {prompt} and your current task is to {current_subgoal}",image_str )
                elif llava_response in VALID_DIRECTIONS:
                    feedback = execute_with_feedback(move_robot, llava_response)
                elif llava_response.startswith("arm pre_grab"):
                    feedback = execute_with_feedback(set_pre_pick)
                elif llava_response.startswith("arm tucked_in"):
                    feedback = execute_with_feedback(set_tucked_in)
                elif llava_response.startswith("arm reach_forward"):
                    distance = float(llava_response.split(" ")[-1])
                    feedback = execute_with_feedback(extend_arm, distance)
                elif llava_response.startswith("arm retract"):
                    distance = float(llava_response.split(" ")[-1])
                    feedback = execute_with_feedback(retract_arm, distance)
                elif llava_response.startswith("arm rotate"):
                    direction = llava_response.split(" ")[-1]
                    feedback = execute_with_feedback(rotate_arm, direction)
                elif llava_response.startswith("head"):
                    direction = llava_response.split(" ")[-1]
                    feedback = execute_with_feedback(move_head, direction)
                elif llava_response.startswith("control_gripper"):
                    action = llava_response.split(" ")[-1]
                    feedback = execute_with_feedback(control_gripper, action)
                else:
                    feedback = "Invalid command. Please try again."
                    llava_control_response = llava_control(f" '{llava_response}' is not a valid robot instruction listed, try again the goal is: {prompt} and your current task is to {current_subgoal}",image_str )

                gemini_response_history.append(f"Feedback: {feedback}")

                if "done" in llava_response.lower():
                    current_subgoal_index += 1
                    if current_subgoal_index >= len(subgoals):
                        stop_gemini = True
                        print("All subgoals completed")
                    else:
                        print("moving to next subgoal")
                # else:
                #     # print what llava response startsd with
                #     print(f" first string is {llava_response.split(' ')[0]}")
                #     #tell the VLm that the response it gave is not
                #     llava_control_response = llava_control(f" '{llava_response}' is not a valid robot instruction listed, try again the goal is: {prompt} and your current task is to {current_subgoal}",image_str )

        rospy.sleep(1)


# endregion llava_control_loop


# region Gemini_flask
@app.route('/send_prompt', methods=['POST'])
async def send_prompt():
    global gemini_response, subgoals, current_subgoal_index
    user_prompt = request.form.get('prompt')

    gemini_response_history.clear() # clearing history
    gemini_response_history.append(f"user: {user_prompt}")

    subgoals = generate_subgoals(user_prompt)
    if subgoals:
        gemini_response_history.append(f"Gemini (subgoals): {subgoals}")

    current_subgoal_index = 0 # reset index

    # asyncio.create_task(llava_control_loop(user_prompt, subgoals))
    await llava_control_loop(user_prompt, subgoals)
    # llava_thread = threading.Thread(target=llava_control_loop, args=(user_prompt, subgoals))
    # llava_thread.start()

    return render_template('index.html',
                           goal_setter_response=subgoals,
                           control_model_response=gemini_response,
                           arm_positions=ARM_POSITIONS.keys(),
                           current_position=get_current_arm_position_name(current_arm_position))

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