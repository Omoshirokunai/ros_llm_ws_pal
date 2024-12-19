import threading
import time
from dataclasses import dataclass
from enum import Enum
from queue import Queue

from rich import print
from ssh_with_ros import PersistentSSHClient, SingleCommandSSHClient


@dataclass
class MovementParameters:
    linear_velocity: float  # m/s
    angular_velocity: float  # rad/s
    duration: float  # seconds

    def __post_init__(self):
        # Safety constraints
        self.linear_velocity = max(min(self.linear_velocity, 0.5), -0.5)
        self.angular_velocity = max(min(self.angular_velocity, 0.5), -0.5)
        self.duration = max(min(self.duration, 5.0), 0.1)

class MovementType(Enum):
    FORWARD = "forward"
    TURN = "turn"
    COMPOUND = "compound"

@dataclass
class SafeMovement:
    movement_type: MovementType
    params: MovementParameters

class RobotControl:
    def __init__(self):
        self.ssh_client = SingleCommandSSHClient()
        # self.ssh_client = PersistentSSHClient()

        self.movement_queue = Queue()
        self.is_executing = False
        self.stop_flag = False
        self.current_movement = None
        self.collision_threshold = 0.3 # 30 cm

        # Start movement execution thread
        self.execution_thread = threading.Thread(target=self._movement_executor)
        self.execution_thread.daemon = True
        self.execution_thread.start()

    def check_collision(self, movement: SafeMovement) -> bool:
        """Check if movement would cause collision using lidar data"""
        # Get LIDAR readings
        lidar_command = "rostopic echo -n1 /scan"
        result = self.ssh_client.execute_command(lidar_command)

        # Parse LIDAR data and check collision potential
        if movement.movement_type == MovementType.FORWARD:
            # Check forward obstacles
            if min(result.ranges[330:30]) < self.collision_threshold:
                return True
        return False


    def execute_movement(self, movement: SafeMovement):
        """Safely execute a movement with continuous collision checking"""
        start_time = time.time()

        while time.time() - start_time < movement.params.duration:
            if self.check_collision(movement):
                self.stop_robot()
                return False

            command = self._movement_to_command(movement)
            self.ssh_client.execute_command(command)
            time.sleep(0.1)

        self.stop_robot()
        return True

    def _movement_to_command(self, movement: SafeMovement) -> str:
        """Convert movement to ROS command"""
        if movement.movement_type == MovementType.FORWARD:
            return f"rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist "\
                   f"'[{movement.params.linear_velocity},0.0,0.0]' '[0.0, 0.0, 0.0]' --once"
        elif movement.movement_type == MovementType.TURN:
            return f"rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist "\
                   f"'[0.0,0.0,0.0]' '[0.0, 0.0, {movement.params.angular_velocity}]' --once"

    def queue_movement(self, movement: SafeMovement):
        """Add movement to queue"""
        self.movement_queue.put(movement)

    def _movement_executor(self):
        """Background thread to execute queued movements"""
        while True:
            if not self.movement_queue.empty() and not self.stop_flag:
                movement = self.movement_queue.get()
                self.current_movement = movement
                self.execute_movement(movement)
                self.current_movement = None
            time.sleep(0.1)

    def move_forward_by(self, distance: float, speed: float = 0.3):
        """Move forward by specified distance"""
        duration = abs(distance / speed)
        movement = SafeMovement(
            movement_type=MovementType.FORWARD,
            params=MovementParameters(
                linear_velocity=speed,
                angular_velocity=0.0,
                duration=duration
            )
        )
        self.queue_movement(movement)

    def turn_by_angle(self, angle: float, speed: float = 0.3):
        """Turn by specified angle in radians"""
        duration = abs(angle / speed)
        movement = SafeMovement(
            movement_type=MovementType.TURN,
            params=MovementParameters(
                linear_velocity=0.0,
                angular_velocity=speed if angle > 0 else -speed,
                duration=duration
            )
        )
        self.queue_movement(movement)

        # self.current_torso_height = 0.0
    def robot_set_home(self):
        command = "rosrun play_motion run_motion home"
        self.ssh_client.execute_command(command)
        # self._execute_command_for_duration(command, 3)
        return True

    def stop_robot(self):
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.0,0.0,0.0]' '[0.0, 0.0, 0.0]' --once"
        self.ssh_client.execute_command(command)
        return True

    # def _execute_timed_command(self, command, duration=2):
    #     try:
    #         self.ssh_client.execute_command(command)
    #         time.sleep(duration)
    #         self.stop_robot()  # Stop after duration
    #     except Exception as e:
    #         print(f"Failed to execute command: {e}")
    #         raise e

    def move_forward(self):
        print("[green] excuting move forward [/green]")
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.5,0.0,0.0]' '[0.0, 0.0, 0.0]' --once"
        self.ssh_client.execute_command(command)
        time.sleep(1)
        return True
        # try:
        #     self.ssh_client.execute_command(command)
        #     time.sleep(4)

        # except Exception as e:
        #     print(f"Failed to connect to SSH: {e}")
        #     raise e
        # return True

    def turn_right(self):
        print("[green] excuting turn right [/green]")
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.0,0.0,0.0]' '[0.0 ,0.0, -0.5]' --once"
        # self._execute_command_for_duration(command, duration)
        self.ssh_client.execute_command(command)
        time.sleep(1)
        return True


    def turn_left(self):
        print("[green] excuting turn left [/green]")
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.0,0.0,0.0]' '[0.0, 0.0, 0.5]' --once"
        # self._execute_command_for_duration(command, duration)
        self.ssh_client.execute_command(command)
        time.sleep(1)

        return True




    def _execute_command_for_duration(self, command, duration):
        import time

        # start_time = time.time()
        end_time = time.time() + duration
        # while (time.time() - start_time) < duration:
        while time.time() < end_time:

            self.ssh_client.execute_command(command)
            time.sleep(0.1)  # Add a small delay to avoid flooding the command