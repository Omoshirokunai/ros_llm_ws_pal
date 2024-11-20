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
def fetch_image_via_ssh(remote_image_path=remote_image_path):
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

# In sensor_data.py

# Add new environment variables at the top with the other env vars:
remote_map_path = os.environ.get("REMOTE_MAP_PATH", "/home/pal/map.jpg")
local_map_path = os.environ.get("LOCAL_MAP_PATH", "map.jpg")
expected_map_size = 100000  # Adjust based on typical map file size

def fetch_map_via_ssh():
    """Fetch the map.jpg file from remote robot via SSH"""
    if not all([ssh_user, ssh_host, password]):
        print("Missing SSH configuration in environment variables.")
        return False

    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh_client.connect(ssh_host, username=ssh_user, port=ssh_port, password=password)
        sftp = ssh_client.open_sftp()

        try:
            while True:
                try:
                    sftp.get(remote_map_path, local_map_path)

                    if os.path.exists(local_map_path) and os.path.getsize(local_map_path) > 0:
                        print(f"Map fetched successfully: {os.path.getsize(local_map_path)} bytes")
                        return True
                except FileNotFoundError:
                    print("Map file not found on remote system, waiting...")
                except Exception as e:
                    print(f"Error fetching map: {e}")

                time.sleep(2)  # Wait before retry

        finally:
            sftp.close()
    except Exception as e:
        print(f"Failed to connect to SSH server: {e}")
        return False
    finally:
        ssh_client.close()



