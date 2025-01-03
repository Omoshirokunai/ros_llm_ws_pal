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
        # self.SSH_PORT = 100

        self.PASS = os.getenv("PASS")
        self.ROS_SETUP_CMD = os.getenv("ROS_SETUP_CMD")

    def execute_command(self, command):
        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            ssh_client.connect(self.SSH_HOST, username=self.SSH_USER, port=self.SSH_PORT, password=self.PASS)
            # print(f"Executing command")
        except Exception as e:
            print(f"Failed to connect to SSH: {e}")
            raise e

        full_command = f"{self.ROS_SETUP_CMD} && {command}"
        # full_command = f"source ~/robotics_ws/devel/setup.bash && {command}"

        ssh_stdin, ssh_stdout, ssh_stderr = ssh_client.exec_command(full_command)
        # print(f"{full_command} executed")
        output = ssh_stdout.read().decode()
        error = ssh_stderr.read().decode()
        # print(output)
        ssh_client.close()
        return output, error


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

    # def execute_command(self, command, using_play_motion=False):
        # if not play_motion:
    def execute_command(self, command):
        ssh_stdin, ssh_stdout, ssh_stderr = self.ssh_client.exec_command(f"{self.ROS_SETUP_CMD} && {command}")
        # else:
        # ssh_stdin, ssh_stdout, ssh_stderr = self.ssh_client.exec_command(f"{command}")
        return ssh_stdout.read().decode(), ssh_stderr.read().decode()

    def close(self):
        self.ssh_client.close()