#--- new
import os
import threading

import paramiko
from dotenv import load_dotenv
from ssh_with_ros import PersistentSSHClient, SingleCommandSSHClient


# Function to fetch the image via SSH
def fetch_image_via_ssh():
    load_dotenv()

    # Ensure environment variables are loaded
    ssh_user = os.getenv("SSH_USER")
    ssh_host = os.getenv("SSH_HOST")
    ssh_port = int(os.getenv("SSH_PORT", 22))
    password = os.getenv("PASS")
    local_image_path =os.environ.get("LOCAL_IMAGE_PATH")
    remote_image_path  = os.environ.get("REMOTE_IMAGE_PATH")

    if not all([ssh_user, ssh_host, password]):
        print("Missing SSH configuration in environment variables.")
        return

    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:

        ssh_client.connect(ssh_host, username=ssh_user, port=ssh_port, password=password)
        sftp = ssh_client.open_sftp()

        try:
            run_camera_capture_script()
            # sftp.get(REMOTE_IMAGE_PATH, LOCAL_IMAGE_PATH)
            sftp.get(remote_image_path, local_image_path)
            # print(f"Failed to fetch image: {e}")
        finally:
            sftp.close()
    except Exception as e:
        print(f"Failed to connect to SSH server: {e}")
    finally:
        ssh_client.close()

# Function to run the camera capture script via SSH
def run_camera_capture_script():
    ##!FIX: command is either not being executed or just locking a thread without doing anything
    ssh_client = SingleCommandSSHClient()
    command = "rosrun get_tiago_camera remote_camera_reader.py"
    # command = "source ~/catkin_ws/devel/setup.bash && bash -c python /home/pal/catkin_ws/src/cam/remote_camera_reader.py'"

    try:
        print(f"Executing command: {command}")
        ssh_client.execute_command(command)
        print(f"Execution complete")

    except Exception as e:
        print(f"Exception while running camera capture script: {e}")

fetch_image_via_ssh()


