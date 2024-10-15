from ssh_with_ros import SingleCommandSSHClient


class RobotControl:
    def __init__(self):
        self.ssh_client = SingleCommandSSHClient()

    def move_forward(self, duration=1):
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.5,0.0,0.0]' '[0.0, 0.0, 0.0]'"
        self._execute_command_for_duration(command, duration)

    def move_right(self, duration=1):
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.0,0.0,-0.5]' '[0.0, 0.0, 0.0]'"
        self._execute_command_for_duration(command, duration)


    def move_left(self, duration=1):
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.0,0.0,0.5]' '[0.0, 0.0, 0.0]'"
        self._execute_command_for_duration(command, duration)

    def torso_up(self, duration=1):
        command = "rostopic pub /torso_controller/command std_msgs/Float64 '1.0'"
        self._execute_command_for_duration(command, duration)

    def torso_down(self, duration=1):
        command = "rostopic pub /torso_controller/command std_msgs/Float64 '0.0'"
        self._execute_command_for_duration(command, duration)

    def _execute_command_for_duration(self, command, duration):
        import time
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.ssh_client.execute_command(command)
            time.sleep(0.1)  # Add a small delay to avoid flooding the command