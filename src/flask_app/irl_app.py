import base64
from io import BytesIO

# import cv2
from flask import Flask, Response, json, jsonify, redirect, render_template, request
from PIL import Image
from robot_control_over_ssh import RobotControl
from sensor_data import RobotSensors
from werkzeug.exceptions import HTTPException

app = Flask(__name__)
robot_control = RobotControl()
robot_sensors = RobotSensors()

@app.route('/')
def index():
    return render_template('irl_index.html')


@app.route('/camera_feed')
def camera_feed():
    frame = robot_sensors.get_camera_data()
    return Response(frame,
                    mimetype='multipart/x-mixed-replace; boundary=frame')

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

# @app.errorhandler(404)
# def page_not_found(e):
#     # note that we set the 404 status explicitly
#     return redirect("/")

# @app.errorhandler(HTTPException)
# def handle_exception(e):
#     """Return JSON instead of HTML for HTTP errors."""
#     # start with the correct headers and status code from the error
#     response = e.get_response()
#     # replace the body with JSON
#     response.data = json.dumps({
#         "code": e.code,
#         "name": e.name,
#         "description": e.description,
#     })
#     response.content_type = "application/json"
#     return response

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)