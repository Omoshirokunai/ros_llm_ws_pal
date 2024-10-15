#Tiago robot control over SSH using Rostopic pub
import os
import subprocess
import time

import paramiko
from dotenv import load_dotenv

load_dotenv()
SSH_USER = os.getenv("SSH_USER")
SSH_HOST = os.getenv("SSH_HOST")
SSH_PORT = int(os.getenv("SSH_PORT"))
PASS = os.getenv("PASS")

ssh_client = paramiko.SSHClient()
ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh_client.connect(SSH_HOST, username=SSH_USER, port=SSH_PORT, password=PASS, timeout=3)
# time.sleep(5)
ssh_stdin, ssh_stdout, ssh_stderr = ssh_client.exec_command("echo $ROS_MASTER_URI")

print(ssh_stdout.read().decode().strip())
# ssh_client.close()
# def run_ros_command(command=):
#     try:
#         ssh_command = f'ssh pal@tiago-hub -p100'