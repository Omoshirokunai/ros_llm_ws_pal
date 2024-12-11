import time

from rich import print
from ssh_with_ros import PersistentSSHClient, SingleCommandSSHClient


class RobotControl:
    def __init__(self):
        self.ssh_client = SingleCommandSSHClient()
        # self.ssh_client = PersistentSSHClient()
        self.head_pose = 0.0
        # self.current_torso_height = 0.0
    def robot_set_home(self):
        command = "rosrun play_motion run_motion home"
        self.ssh_client.execute_command(command)
        # self._execute_command_for_duration(command, 3)
        return True

    def move_forward(self):
        print("[green] excuting move forward [/green]")
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.5,0.0,0.0]' '[0.0, 0.0, 0.0]' -r 2"
        # self._execute_command_for_duration(command, 100)
        try:
            self.ssh_client.execute_command(command)
            time.sleep(1)
        except Exception as e:
            print(f"Failed to connect to SSH: {e}")
            raise e
        return True

    def turn_right(self):
        print("[green] excuting turn right [/green]")
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.0,0.0,0.0]' '[0.0 ,0.0, -0.6]' -r 2"
        # self._execute_command_for_duration(command, duration)
        self.ssh_client.execute_command(command)
        time.sleep(1)
        return True


    def turn_left(self):
        print("[green] excuting turn left [/green]")
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.0,0.0,0.0]' '[0.0, 0.0, 0.6]' -r 2"
        # self._execute_command_for_duration(command, duration)
        self.ssh_client.execute_command(command)
        time.sleep(1)

        return True

    def head_up(self, increment=0.3):
        print("[green] excecuting head up [/green]")
        self.head_pose = min(self.head_pose + increment, -1)
        self.head_pose = max(self.head_pose, 0.7)
        command = f"rosrun play_motion move_joint head_2_joint {self.head_pose} 2.0"
        self.ssh_client.execute_command(command)
        return True


    def _execute_command_for_duration(self, command, duration):
        import time

        # start_time = time.time()
        end_time = time.time() + duration
        # while (time.time() - start_time) < duration:
        while time.time() < end_time:

            self.ssh_client.execute_command(command)
            time.sleep(0.1)  # Add a small delay to avoid flooding the command