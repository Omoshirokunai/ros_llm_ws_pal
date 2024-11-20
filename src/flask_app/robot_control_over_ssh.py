from ssh_with_ros import PersistentSSHClient, SingleCommandSSHClient


class RobotControl:
    def __init__(self):
        # self.ssh_client = SingleCommandSSHClient()
        self.ssh_client = PersistentSSHClient()
        self.head_pose = 0.0
        # self.current_torso_height = 0.0
    def robot_set_home(self, duration=3000):
        command = "rosrun play_motion run_motion home"
        self.ssh_client.execute_command(command)
        # self._execute_command_for_duration(command, 3)

    def move_forward(self):
        print("excuting")
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.5,0.0,0.0]' '[0.0, 0.0, 0.0]'"
        # self._execute_command_for_duration(command, duration)
        self.ssh_client.execute_command(command)

    def turn_right(self, duration=4000):
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.0,0.0,0.0]' '[0.0 ,0.0, -0.4]'"
        # self._execute_command_for_duration(command, duration)
        self.ssh_client.execute_command(command)


    def turn_left(self, duration=4000):
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.0,0.0,0.5]' '[0.0, 0.0, 0.4]'"
        # self._execute_command_for_duration(command, duration)
        self.ssh_client.execute_command(command)

    def head_up(self, increment=0.3):
        self.head_pose = min(self.head_pose + increment, -1)
        self.head_pose = max(self.head_pose, 0.7)
        command = f"rosrun play_motion move_joint head_2_joint {self.head_pose} 2.0"
        self.ssh_client.execute_command(command)


    # def _execute_command_for_duration(self, command, duration):
    #     import time

    #     # start_time = time.time()
    #     end_time = time.time() + duration
    #     # while (time.time() - start_time) < duration:
    #     while time.time() < end_time:

    #         self.ssh_client.execute_command(command)
    #         time.sleep(0.1)  # Add a small delay to avoid flooding the command