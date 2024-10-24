#--- new
import os
import threading
import time

import paramiko
from dotenv import load_dotenv
from ssh_with_ros import PersistentSSHClient, SingleCommandSSHClient

# SSH command to run the start_camera_capture.sh
load_dotenv()

# Ensure environment variables are loaded
ssh_user = os.getenv("SSH_USER")
ssh_host = os.getenv("SSH_HOST")
ssh_port = int(os.getenv("SSH_PORT"))
password = os.getenv("PASS")
local_image_path =os.environ.get("LOCAL_IMAGE_PATH")
remote_image_path  = os.environ.get("REMOTE_IMAGE_PATH")
run_capture_script = os.environ.get("RUN_CAPTURE_SCRIPT")
stop_capture_script =os.environ.get("STOP_CAPTURE_SCRIPT")
expected_image_size = 120529

# Function to trigger the bash script via SSH
def trigger_capture_script():
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    print("starting")
    ssh_client.connect(ssh_host, username=ssh_user, port=ssh_port, password=password)
    try:
        # Run the camera capture script
        stdin, stdout, stderr = ssh_client.exec_command(run_capture_script)
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


# Function to trigger the stop script via SSH
def trigger_stop_script():
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    print("stopping")
    # ssh_client.connect(SSH_HOST, username=SSH_USER, port=SSH_PORT, password='your_password')

    try:
        ssh_client.connect(ssh_host, username=ssh_user, port=ssh_port, password=password)

        # Run the stop script
        stdin, stdout, stderr = ssh_client.exec_command(stop_capture_script)
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
    # time.sleep(0.3)
    if not all([ssh_user, ssh_host, password]):
        print("Missing SSH configuration in environment variables.")
        return

    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:

        ssh_client.connect(ssh_host, username=ssh_user, port=ssh_port, password=password)
        sftp = ssh_client.open_sftp()

        try:
            # run_camera_capture_script()
            # sftp.get(REMOTE_IMAGE_PATH, LOCAL_IMAGE_PATH)
            # sftp.get(remote_image_path, local_image_path)
            while True:
                sftp.get(remote_image_path, local_image_path, max_concurrent_prefetch_requests=20)

                if os.path.getsize(local_image_path) >= expected_image_size:
                    print(os.path.getsize(local_image_path))
                    break
                time.sleep(6)  # Wait for 1 second before checking again
            time.sleep(6)

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
# trigger_capture_script()

# fetch_image_via_ssh()
# trigger_stop_script()

