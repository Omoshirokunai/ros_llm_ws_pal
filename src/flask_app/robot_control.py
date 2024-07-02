# region Imports
import rospy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# endregion Imports


# region Initialize ROS node and publishers
rospy.init_node('robot_control', anonymous=True)
cmd_vel_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
torso_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10)
arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
# endregion Initialize ROS node and publishers

# region Movement
def move_robot(direction):
    """
    Move the robot in the specified direction.
    """
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
# endregion Movement

# region Torso
def update_torso(position):
    """
    Update the position of the robot's torso.
    """
    traj = JointTrajectory()
    traj.joint_names = ['torso_lift_joint']
    point = JointTrajectoryPoint()
    point.positions = [position]
    point.time_from_start = rospy.Duration(1.0)
    traj.points.append(point)

    torso_pub.publish(traj)
# endregion Torso

# region arm
def update_arm(positions):
    traj = JointTrajectory()
    traj.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']

    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(1.0)
    traj.points.append(point)

    arm_pub.publish(traj)
# endregion arm
