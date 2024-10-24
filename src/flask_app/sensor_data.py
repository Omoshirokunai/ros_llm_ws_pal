#--- new
import os
import threading

import paramiko
from dotenv import load_dotenv
from ssh_with_ros import PersistentSSHClient, SingleCommandSSHClient

# SSH command to run the start_camera_capture.sh
RUN_CAPTURE_SCRIPT = "/path/to/start_camera_capture.sh"

# Function to trigger the bash script via SSH
def trigger_capture_script():
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    ssh_client.connect(SSH_HOST, username=SSH_USER, port=SSH_PORT, password='your_password')

    try:
        # Run the camera capture script
        stdin, stdout, stderr = ssh_client.exec_command(RUN_CAPTURE_SCRIPT)
        output = stdout.read().decode()
        errors = stderr.read().decode()

        if errors:
            print(f"Error running capture script: {errors}")
            return False
        else:
            print(f"Capture script executed: {output}")
            return True
    finally:
        ssh_client.close()
# SSH command to run the stop_camera_capture.sh
STOP_CAPTURE_SCRIPT = "/path/to/stop_camera_capture.sh"  # Update this path

# Function to trigger the stop script via SSH
def trigger_stop_script():
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    ssh_client.connect(SSH_HOST, username=SSH_USER, port=SSH_PORT, password='your_password')

    try:
        # Run the stop script
        stdin, stdout, stderr = ssh_client.exec_command(STOP_CAPTURE_SCRIPT)
        output = stdout.read().decode()
        errors = stderr.read().decode()

        if errors:
            print(f"Error stopping capture script: {errors}")
            return False
        else:
            print(f"Stop script executed: {output}")
            return True
    finally:
        ssh_client.close()

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

# # Function to run the camera capture script via SSH
# def run_camera_capture_script():
#     ##!FIX: command is either not being executed or just locking a thread without doing anything
#     ssh_client = SingleCommandSSHClient()
#     command = "rosrun get_tiago_camera remote_camera_reader.py"
#     # command = "source ~/catkin_ws/devel/setup.bash && bash -c python /home/pal/catkin_ws/src/cam/remote_camera_reader.py'"

#     try:
#         print(f"Executing command: {command}")
#         ssh_client.execute_command(command)
#         print(f"Execution complete")

#     except Exception as e:
#         print(f"Exception while running camera capture script: {e}")

fetch_image_via_ssh()


