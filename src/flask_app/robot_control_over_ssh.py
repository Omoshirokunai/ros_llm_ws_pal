import json
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
        self.duration = max(min(self.duration, 4.0), 0.1)

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
        self.collision_threshold = 0.03 # 30 cm

        # Start movement execution thread
        self.execution_thread = threading.Thread(target=self._movement_executor)
        self.execution_thread.daemon = True
        self.execution_thread.start()

    def check_collision(self, movement: SafeMovement) -> bool:
        """Check if movement would cause collision using lidar data"""
        # Get LIDAR readings
        lidar_command = "rostopic echo -n1 /scan"
        result = self.ssh_client.execute_command(lidar_command)

        try:
            # Get laser scan data via SSH
            cmd = "rostopic echo -n1 /scan"
            result = self.ssh_client.execute_command(cmd)

            # Parse the string output into usable data
            if isinstance(result, str):
                # Split the output into lines and find ranges line
                lines = result.split('\n')
                ranges_line = None
                for line in lines:
                    if 'ranges:' in line:
                        ranges_line = line.replace('ranges:', '').strip()
                        break

                if ranges_line:
                    # Convert string of ranges to float list
                    ranges = [float(x) for x in ranges_line.strip('[]').split(',') if x.strip()]

                    # Check for obstacles within safety threshold
                    MIN_DISTANCE = 0.5  # meters
                    if any(r < MIN_DISTANCE for r in ranges if r > 0.0):
                        return True  # Collision risk detected

                return False  # No collision risk

        except Exception as e:
            print(f"Error checking collisions: {e}")
            return True  # Return True as safety measure
        # try:
        #     # with open("src/flask_app/static/images/lidar_data.json", "r") as f:
        #     #     result = json.loads(f.read())

        #     ranges = result[0].ranges

        #     # Parse LIDAR data and check collision potential
        #     if movement.movement_type == MovementType.FORWARD:
        #         # Check forward obstacles
        #         center_idx = len(ranges) // 2  # 90° position
        #         cone_width = len(ranges) // 12  # ±15° on each side
        #         forward_ranges = ranges[center_idx - cone_width : center_idx + cone_width]

        #         if min(forward_ranges) < self.collision_threshold:
        #             return True
        #         return False
        # except (FileNotFoundError, json.JSONDecodeError, KeyError, ValueError) as e:
        #     print(f"Failed to read LIDAR data: {e}")
        #     return True  # Fail safe
        #     if min(result.ranges[180:30]) < self.collision_threshold:
        #         return True
        # return False


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
        try:
            distance  = distance / 10
            distance = max(min(distance, 2.0), -2.0)
            speed = max(min(speed, 0.5), -0.5)
            duration = abs(distance / speed)

            print(distance, speed, duration)

            movement = SafeMovement(
                movement_type=MovementType.FORWARD,
                params=MovementParameters(
                    linear_velocity=speed,
                    angular_velocity=0.0,
                    duration=duration
                )
            )
            self.queue_movement(movement)
        except Exception as e:
            print(f"Failed to move forward: {e}")
            return False
        return True

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
        return True


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


    def move_forward(self):
        print("[green] excuting move forward [/green]")
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.5,0.0,0.0]' '[0.0, 0.0, 0.0]' --once"
        self.ssh_client.execute_command(command)
        time.sleep(1)
        return True


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