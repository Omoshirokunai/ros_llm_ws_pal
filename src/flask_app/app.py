from flask import Flask, jsonify, request, render_template_string, render_template, redirect, url_for, Response
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import cv2

app = Flask(__name__)
rospy.init_node('flask_controller', anonymous=True)
cmd_vel_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

lidar_data = None

def lidar_callback(data):
    global lidar_data
    lidar_data = data
def generate_frames():
    # Initialize the camera
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    cap.release()
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def publish_twist(linear_x=0, linear_y=0, linear_z=0, angular_x=0, angular_y=0, angular_z=0):
    global cmd_vel_pub
    twist = Twist()
    twist.linear.x = linear_x
    twist.linear.y = linear_y
    twist.linear.z = linear_z
    twist.angular.x = angular_x
    twist.angular.y = angular_y
    twist.angular.z = angular_z
    cmd_vel_pub.publish(twist)
    
def ros_listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.spin()
    
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/lidar')
def get_lidar_data():
    global lidar_data
    if lidar_data:
        return jsonify({
            'ranges': lidar_data.ranges,

        })
    else:
        return jsonify({'error': 'No data received'}), 500
@app.route('/move', methods=['POST'])
def move():
    direction = request.form['direction']
    twist = Twist()

    if direction == 'forward':
        twist.linear.x = 0.5
    elif direction == 'backward':
        twist.linear.x = -0.5
    elif direction == 'left':
        twist.angular.z = 0.5
    elif direction == 'right':
        twist.angular.z = -0.5

    cmd_vel_publisher.publish(twist)
    return '', 204
# @app.route('/move_forward', methods=['POST'])
# def move_forward():
#     publish_twist(linear_x=1)
#     print("forward")
#     return redirect(url_for('index'))
# @app.route('/move_backward', methods=['POST'])
# def move_backward():
#     publish_twist(linear_x=-0.5)
#     return redirect(url_for('index'))

# @app.route('/turn_right', methods=['POST'])
# def turn_right():
#     publish_twist(angular_z=0.5)
#     return redirect(url_for('index'))

# @app.route('/turn_left', methods=['POST'])
# def turn_left():
#     publish_twist(angular_z=-0.5)
#     return redirect(url_for('index'))

# @app.route('/stop', methods=['POST'])
# def stop():
#     publish_twist()
#     return redirect(url_for('index'))

if __name__ == '__main__':
    # rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber('/scan_raw', LaserScan, lidar_callback)
    # cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    app.run(debug=True, host='0.0.0.0')
