# region Imports
import rospy  # type: ignore
from control_msgs.msg import JointTrajectoryControllerState  # type: ignore
from geometry_msgs.msg import Twist  # type: ignore
from nav_msgs.msg import Odometry  # type: ignore
from std_msgs.msg import Float64  # type: ignore
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # type: ignore

# import asyncio
# from sensor_msgs.msg import JointState # type: ignore
# endregion Imports


# region Initialize ROS node and publishers
rospy.init_node('robot_control', anonymous=True)
cmd_vel_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
torso_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10)
arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)
gripper_pub = rospy.Publisher('/gripper_controller/command',JointTrajectory, queue_size=10)
VALID_DIRECTIONS = ['forward', 'backward', 'left', 'right']
# endregion Initialize ROS node and publishers

# region Movement
def move_robot(direction:str)->str:
    """
    Move the robot in the specified direction.
    """
    twist = Twist()
    assert direction in VALID_DIRECTIONS, "Invalid direction"
    print(f"moving {direction}")
    if direction == 'forward':
        twist.linear.x = 0.8
        twist.angular.z = 0

    elif direction == 'backward':
        twist.linear.x = -0.9
        twist.angular.z = 0

    elif direction == 'left':
        twist.angular.z = 0.5
        # twist.linear.x = 0.8
    elif direction == 'right':
        twist.angular.z = -0.5
        # twist.linear.x = 0.5

    # rospy.sleep(1)
    # cmd_vel_publisher.publish(twist)
    # rospy.sleep(3.0)
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < 0.9:
        cmd_vel_publisher.publish(twist)
        rospy.sleep(0.1)

    twist.linear.x = 0
    twist.angular.z = 0
    cmd_vel_publisher.publish(twist)

    return f"Moved {direction}"

# endregion Movement



# region arm


def update_arm(positions: 'list[float]')->str:
    traj = JointTrajectory()
    traj.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']

    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(1.0)
    traj.points.append(point)

    arm_pub.publish(traj)
    rospy.sleep(1)
    # return "Arm position updated"
    current_arm_position = rospy.wait_for_message('/arm_controller/state', JointTrajectoryControllerState, timeout=3)
    if current_arm_position:
        return f"Arm updated. New positions: {[f'{p:.2f}' for p in current_arm_position.actual.positions]}"
    else:
        return "Arm movement completed, but couldn't get position feedback."
# endregion arm

def set_pre_pick():
    pre_pick_positions = [0.08, 0.8, -1.7, 1.5, 0.0, 0.2, 0.0]
    return update_arm(pre_pick_positions)

def set_tucked_in():
    tucked_in_positions = [1.2, 0.2, 0.0, -0.5, 0.0, -0.8, 0.0]
    return update_arm(tucked_in_positions)

def extend_arm(distance=0.02):
    current_arm_position = rospy.wait_for_message('/arm_controller/state', JointTrajectoryControllerState, timeout=3)
    if current_arm_position is None:
        return "Failed to get current arm position"

    new_positions = list(current_arm_position.actual.positions)
    new_positions[0] += distance
    return update_arm(new_positions)

def retract_arm(distance=0.1):
    current_arm_position = rospy.wait_for_message('/arm_controller/state', JointTrajectoryControllerState, timeout=3)
    if current_arm_position is None:
        return "Failed to get current arm position"

    new_positions = list(current_arm_position.actual.positions)
    new_positions[0] -= distance
    return update_arm(new_positions)

def rotate_arm(direction):
    current_arm_position = rospy.wait_for_message('/arm_controller/state', JointTrajectoryControllerState, timeout=3)
    if current_arm_position is None:
        return "Current arm position unknown"
    new_positions = list(current_arm_position.actual.positions)
    if direction == 'clockwise':
        # current_arm_position[2] -= 0.1
        new_positions[2] -= 0.1
    elif direction == 'anticlockwise':
        # current_arm_position[2] += 0.1
        new_positions[2] += 0.1

    return update_arm(current_arm_position)

# endregion Arm Movements

# region Head Movement
def move_head(direction: str)->str:
    """ Move the robot's head in the specified direction.

    Args:
        direction (str): The direction to move the head ('up', 'down', 'left', 'right').

    Returns:
        str: A message indicating the result of the head movement.
    """
    traj = JointTrajectory()
    traj.joint_names = ['head_1_joint', 'head_2_joint']

    current_head_position = rospy.wait_for_message('/head_controller/state', JointTrajectoryControllerState, timeout=3)
    if current_head_position is None:
        return "Failed to get current head position"

    point = JointTrajectoryPoint()
    point.positions = list(current_head_position.actual.positions)
    # if head at madxximum position, return head is at maximum tilt try move body instead
    if point.positions[1] == 0.5 and direction == 'up':
        return "Head is at maximum tilt. Try moving the body instead."
    elif point.positions[1] == -0.5 and direction == 'down':
        return "Head is at maximum tilt. Try moving the body instead."
    elif point.positions[0] == 1.0 and direction == 'left':
        # move body instead
        move_robot('left')
        return "Head is at maximum pan. Moved body instead."
    elif point.positions[0] == -1.0 and direction == 'right':
        # move body instead
        move_robot('right')
        return "Head is at maximum pan. Moved body instead."
    else:
        if direction == 'up':
            point.positions[1] = min(point.positions[1] + 0.1, 0.5)
        elif direction == 'down':
            point.positions[1] = max(point.positions[1] - 0.1, -0.5)
        elif direction == 'left':
            point.positions[0] = min(point.positions[0] + 0.1, 1.0)
        elif direction == 'right':
            point.positions[0] = max(point.positions[0] - 0.1, -1.0)
        else:
            return "Invalid direction"

    point.time_from_start = rospy.Duration(1.0)
    traj.points.append(point)

    head_pub.publish(traj)
    rospy.sleep(2)
    return f"Head moved {direction}. New position: pan={point.positions[0]:.2f}, tilt={point.positions[1]:.2f}"
# endregion Head Movement

# region Gripper
def control_gripper(open_close: str)->str:
    """
    Open or close the gripper to a specified width.
    """
    current_width = rospy.get_param('/gripper_controller/max_opening', 0.08)
    if open_close == 'open':
        width = min(current_width + 0.02, 0.08)  # Increase by 2cm, max 8cm
    elif open_close == 'close':
        width = max(current_width - 0.02, 0.0)  # Decrease by 2cm, min 0cm
    else:
        return "Invalid gripper command"

    gripper_pub.publish(width)
    rospy.sleep(2)
    return f"Gripper {open_close}ed to width: {width:.2f}m"

# endregion Gripper

# region Feedback
def execute_with_feedback(func, *args):
    result = func(*args)
    print(result)
    return result
# endregion Feedback