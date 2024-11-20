## connect to robot via ssh
#execute command to move the robot forward

import os

import paramiko
from dotenv import load_dotenv


def run_command(command):
    load_dotenv()
    SSH_USER = os.getenv("SSH_USER")
    SSH_HOST = os.getenv("SSH_HOST")
    SSH_PORT = int(os.getenv("SSH_PORT"))
    PASS = os.getenv
    ROS_SETUP_CMD = os.getenv("ROS_SETUP_CMD", "source ~/.bashrc")
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh_client.connect(SSH_HOST, username=SSH_USER, port=SSH_PORT, password=PASS)
        print(f"Executing command: {command}")
    except Exception as e:
        print(f"Failed to connect to SSH: {e}")
        raise e

    full_command = f"{ROS_SETUP_CMD} && {command}"
    ssh_stdin, ssh_stdout, ssh_stderr = ssh_client.exec_command(full_command)
    output = ssh_stdout.read().decode()

    error = ssh_stderr.read().decode()
    print(output)
    ssh_client.close()
    return output, error

command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.5,0.0,0.0]' '[0.0, 0.0, 0.0]'"

run_command(command)