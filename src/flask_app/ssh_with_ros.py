#Tiago robot control over SSH using Rostopic pub
import os
import subprocess
import time

import paramiko
from dotenv import load_dotenv

# import warnings



class SingleCommandSSHClient:
    """will be used for sending single use commands that run over a short duration eg movement and joint control
    """
    def __init__(self):
        load_dotenv()
        self.SSH_USER = os.getenv("SSH_USER")
        self.SSH_HOST = os.getenv("SSH_HOST")
        self.SSH_PORT = int(os.getenv("SSH_PORT"))
        self.PASS = os.getenv("PASS")
        self.ROS_SETUP_CMD = os.getenv("ROS_SETUP_CMD", "source ~/.bashrc")

    def execute_command(self, command):
        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh_client.connect(self.SSH_HOST, username=self.SSH_USER, port=self.SSH_PORT, password=self.PASS)
        ssh_stdin, ssh_stdout, ssh_stderr = ssh_client.exec_command(f"{self.ROS_SETUP_CMD} && {command}")
        output = ssh_stdout.read().decode()
        ssh_client.close()
        return output

class PersistentSSHClient:
    """will be used ot get senor data like camera and lidar
    """
    def __init__(self):
        load_dotenv()
        self.SSH_USER = os.getenv("SSH_USER")
        self.SSH_HOST = os.getenv("SSH_HOST")
        self.SSH_PORT = int(os.getenv("SSH_PORT"))
        self.PASS = os.getenv("PASS")
        self.ROS_SETUP_CMD = os.getenv("ROS_SETUP_CMD", "source ~/.bashrc")
        self.ssh_client = paramiko.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh_client.connect(self.SSH_HOST, username=self.SSH_USER, port=self.SSH_PORT, password=self.PASS)

    def execute_command(self, command):
        ssh_stdin, ssh_stdout, ssh_stderr = self.ssh_client.exec_command(f"{self.ROS_SETUP_CMD} && {command}")
        return ssh_stdout.read().decode(), ssh_stderr.read().decode()

    def close(self):
        self.ssh_client.close()
# load_dotenv()
# SSH_USER = os.getenv("SSH_USER")
# SSH_HOST = os.getenv("SSH_HOST")
# SSH_PORT = int(os.getenv("SSH_PORT"))
# PASS = os.getenv("PASS")
# ROS_SETUP_CMD = os.getenv("ROS_SETUP_CMD", "source ~/.bashrc")
# ssh_client = paramiko.SSHClient()
# ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
# # ssh_client.load_system_host_keys()
# ssh_client.connect(SSH_HOST, username=SSH_USER, port=SSH_PORT, password=PASS)
# time.sleep(5)
# try:
#     ssh_stdin, ssh_stdout, ssh_stderr = ssh_client.exec_command(f"{ROS_SETUP_CMD} && $ROS_MASTER_URI")
#     print(ssh_stdout.read().decode())

#     def move_forward(duration=0.9):
#         command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.5,0.0,0.0]' '[0.0, 0.0, 0.0]'"
#         start_time = time.time()
#         while (time.time() - start_time) < duration:
#             ssh_stdin, ssh_stdout, ssh_stderr = ssh_client.exec_command(f"{ROS_SETUP_CMD} && {command}")
#             time.sleep(0.1)  # Add a small delay to avoid flooding the command

#     move_forward()
# finally:
# # print(ssh_stdout.read().decode().strip())
#     ssh_client.close()

