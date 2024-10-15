#Tiago robot control over SSH using Rostopic pub
import os
import subprocess
import time
import warnings

import paramiko
from dotenv import load_dotenv

load_dotenv()
SSH_USER = os.getenv("SSH_USER")
SSH_HOST = os.getenv("SSH_HOST")
SSH_PORT = int(os.getenv("SSH_PORT"))
PASS = os.getenv("PASS")
ROS_SETUP_CMD = os.getenv("ROS_SETUP_CMD", "source ~/.bashrc")
ssh_client = paramiko.SSHClient()
ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
# ssh_client.load_system_host_keys()
ssh_client.connect(SSH_HOST, username=SSH_USER, port=SSH_PORT, password=PASS)
# time.sleep(5)
try:
    ssh_stdin, ssh_stdout, ssh_stderr = ssh_client.exec_command(f"{ROS_SETUP_CMD} && $ROS_MASTER_URI")
    print(ssh_stdout.read().decode())

    def move_forward(duration=0.9):
        command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.5,0.0,0.0]' '[0.0, 0.0, 0.0]'"
        start_time = time.time()
        while (time.time() - start_time) < duration:
            ssh_stdin, ssh_stdout, ssh_stderr = ssh_client.exec_command(f"{ROS_SETUP_CMD} && {command}")
            time.sleep(0.1)  # Add a small delay to avoid flooding the command

    move_forward()
finally:
# print(ssh_stdout.read().decode().strip())
    ssh_client.close()

