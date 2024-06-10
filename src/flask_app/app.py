
from flask import Flask, jsonify, request,stream_with_context, render_template_string, render_template, redirect, url_for, Response
import rospy
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge, CvBridgeError
import threading

#* flask and ROS config
app = Flask(__name__)
rospy.init_node('flask_controller', anonymous=True)
cmd_vel_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
bridge = CvBridge()
latest_frame = None
lock = threading.Lock()
#* home
@app.route('/')
def index():
    return render_template('index.html')

#* camera
def image_callback(msg):
    global latest_frame
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # Optionally resize the image for better performance
        resized_image = cv2.resize(cv_image, (640, 480))
        with lock:
            latest_frame = resized_image
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))    
# Subscribe to the camera topic
rospy.Subscriber('/xtion/rgb/image_raw', Image, image_callback, queue_size=10)

def generate_frames():
    global latest_frame
    while True:
        with lock:
            if latest_frame is not None:
                ret, buffer = cv2.imencode('.jpg', latest_frame)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        rospy.sleep(0.1)        
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

#* lidar section
lidar_data = None

def lidar_callback(data):
    global lidar_data
    lidar_data = data    
def ros_listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.spin()
    

@app.route('/lidar')
def get_lidar_data():
    global lidar_data
    if lidar_data:
        return jsonify({
            'ranges': lidar_data.ranges,

        })
    else:
        return jsonify({'error': 'No data received'}), 500
    
#* movement section
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
    
@app.route('/move', methods=['POST'])
def move():
    direction = request.form['direction']
    twist = Twist()

    if direction == 'forward':
        twist.linear.x = 1
    elif direction == 'backward':
        twist.linear.x = -1
    elif direction == 'left':
        twist.angular.z = 1
    elif direction == 'right':
        twist.angular.z = -1

    cmd_vel_publisher.publish(twist)
    return '', 204

if __name__ == '__main__':
    
    rospy.Subscriber('/scan_raw', LaserScan, lidar_callback)
    app.run(debug=True, host='0.0.0.0')
