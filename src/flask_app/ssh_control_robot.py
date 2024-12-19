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
    ROS_IP_CMD = os.getenv("ROS_IP_CMD", "hostname -I | cut -d' ' -f1")
    ROS_MASTER_URI_CMD = os.getenv("ROS_MASTER_URI_CMD", "echo $ROS_MASTER_URI")
    PASS = os.getenv("PASS")
    ROS_SETUP_CMD = os.getenv("ROS_SETUP_CMD", "source ~/.bashrc")
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh_client.connect(hostname=SSH_HOST, username=SSH_USER, port=SSH_PORT, password=PASS)

    except Exception as e:
        print(f"Failed to connect to SSH: {e}")
        raise e

    #  # Wait for roscore to start
    # time.sleep(5)
    # full_command = f"source ~/catkin_ws/devel/setup.bash && {command}"
    # full_command = f"{ROS_SETUP_CMD} && {command}"

    full_command = f"source ~/robotics_ws/devel/setup.bash && {command}"

    # full_command = f"source ~/.bashrc && {command}"


    ssh_stdin, ssh_stdout, ssh_stderr = ssh_client.exec_command(full_command)
    # time.sleep(4)

    # Check for errors
    error = ssh_stderr.read().decode()
    if error:
            # print(f"Error: {error}")
            raise Exception(f"Error: {error}")
            # return None
    output = ssh_stdout.read().decode()

    print(output)
    ssh_client.close()
    return output

if __name__ == "__main__":
    # command = "rostopic list"
    run_command("echo $ROS_MASTER_URI && echo $ROS_HOSTNAME && echo $ROS_IP")
    # Test ROS communication
    run_command("rostopic list")
    # run_command("echo $ROS_IP")
    # run_command("rostopic echo /scan_raw -n 1")
    # cmd_vel_command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' --once"
    # run_command(cmd_vel_command)
    # command = "echo $ROS_MASTER_URI"
    command = "rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist '[0.0,0.0,0.0]' '[0.0, 0.0, 0.4]' --once"

    run_command(command)
    time.sleep(0.1)
