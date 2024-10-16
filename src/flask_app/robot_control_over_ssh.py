from ssh_with_ros import SingleCommandSSHClient


class RobotControl:
    def __init__(self):
        self.ssh_client = SingleCommandSSHClient()
        # self.current_torso_height = 0.0
    def robot_set_home(self, duration=3000):
        command = "rosrun play_motion run_motion home"
        self.ssh_client.execute_command(command)
        # self._execute_command_for_duration(command, 3)

    def move_forward(self, duration=200):
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.5,0.0,0.0]' '[0.0, 0.0, 0.0]'"
        self._execute_command_for_duration(command, duration)

    def turn_right(self, duration=4000):
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.0,0.0,0.0]' '[0.0 ,0.0, -0.4]'"
        self._execute_command_for_duration(command, duration)


    def turn_left(self, duration=4000):
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.0,0.0,0.5]' '[0.0, 0.0, 0.4]'"
        self._execute_command_for_duration(command, duration)

    def head_up(self, duration=4000):
        command = "rosrun play_motion move_joint head_2_joint -0.1 2.0"
        self.ssh_client.execute_command(command)

    def pre_grasp(self, duration=2):
        pass
        # pre_pick_positions = [0.08, 0.8, -1.7, 1.5, 0.0, 0.2, 0.0]

        # arm_1_position = "rosrun play_motion move_joint arm_1_joint 0.08 2.0"
        # arm_2_position = "rosrun play_motion move_joint arm_2_joint 0.8 2.0"
        # arm_3_position = "rosrun play_motion move_joint arm_3_joint -1.7 2.0"
        # arm_4_position = "rosrun play_motion move_joint arm_4_joint 0.4 2.0"
        # arm_5_position = "rosrun play_motion move_joint arm_5_joint 0.4 2.0"
        # arm_6_position = "rosrun play_motion move_joint arm_6_joint 0.4 2.0"
        # arm_7_position = "rosrun play_motion move_joint arm_7_joint 0 2.0"

        # # joint_positions = "[0.0, -1.0, 1.5, -1.0, 0.0, 1.0, 0.0]"
        # command = f"{arm_1_position}&&{arm_2_position}&&{arm_2_position}&&{arm_7_position}"
        # self._execute_command_for_duration(command, duration)

    def set_torso_home(self, height=0):
        command = "rostopic pub /torso_controller/command std_msgs/Float64 '0'"
        print(self.current_torso_height)
        self._execute_command_for_duration(command, 4)
    # def torso_up(self, increment=0.03):
    #     self.current_torso_height = min(self.current_torso_height + increment, 0.35)
    #     command = "rostopic pub /torso_controller/command std_msgs/Float64 '{self.current_torso_height}'"
    #     print(self.current_torso_height)
    #     self._execute_command_for_duration(command, 4)

    # def torso_down(self, decrement=0.03):
    #     self.current_torso_height = max(self.current_torso_height - decrement, 0)
    #     print(self.current_torso_height)

    #     command = "rostopic pub /torso_controller/command std_msgs/Float64 '{self.current_torso_height}'"
    #     self._execute_command_for_duration(command, 4)

    def _execute_command_for_duration(self, command, duration):
        import time

        # start_time = time.time()
        end_time = time.time() + duration
        # while (time.time() - start_time) < duration:
        while time.time() < end_time:

            self.ssh_client.execute_command(command)
            time.sleep(0.1)  # Add a small delay to avoid flooding the command