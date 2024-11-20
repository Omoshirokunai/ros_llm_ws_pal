## connect to robot via ssh
#execute command to move the robot forward

import os
import time

import paramiko
from dotenv import load_dotenv


def run_command(command):
    load_dotenv()
    SSH_USER = os.getenv("SSH_USER")
    SSH_HOST = os.getenv("SSH_HOST")
    SSH_PORT = int(os.getenv("SSH_PORT"))
    PASS = os.getenv("PASS")
    ROS_SETUP_CMD = os.getenv("ROS_SETUP_CMD", "source ~/.bashrc")
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    ssh_client.connect(hostname=SSH_HOST, username=SSH_USER, port=SSH_PORT, password=PASS)


    #  # Wait for roscore to start
    # time.sleep(5)

    full_command = f"{ROS_SETUP_CMD} && {command}"
    # full_command = f"source ~/.bashrc && {command}"


    ssh_stdin, ssh_stdout, ssh_stderr = ssh_client.exec_command(full_command, get_pty=True, timeout=5)

    # Check for errors
    error = ssh_stderr.read().decode()
    if error:
            print(f"Error: {error}")
            return None
    output = ssh_stdout.read().decode()
    print(output)

    return output


command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.5,0.0,0.0]' '[0.0, 0.0, 0.0]'"

run_command(command)