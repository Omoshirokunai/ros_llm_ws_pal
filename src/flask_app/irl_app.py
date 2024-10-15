from flask import Flask, jsonify, render_template, request
from robot_control_over_ssh import RobotControl

app = Flask(__name__)
robot_control = RobotControl()

@app.route('/')
def index():
    return render_template('irl_index.html')

@app.route('/move_forward', methods=['POST'])
def move_forward():
    robot_control.move_forward()
    return jsonify({"status": "success", "action": "move_forward"})

@app.route('/move_right', methods=['POST'])
def move_right():
    robot_control.move_right()
    return jsonify({"status": "success", "action": "move_right"})

@app.route('/move_left', methods=['POST'])
def move_left():
    robot_control.move_left()
    return jsonify({"status": "success", "action": "move_left"})

@app.route('/torso_up', methods=['POST'])
def torso_up():
    robot_control.torso_up()
    return jsonify({"status": "success", "action": "torso_up"})

@app.route('/torso_down', methods=['POST'])
def torso_down():
    robot_control.torso_down()
    return jsonify({"status": "success", "action": "torso_down"})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)