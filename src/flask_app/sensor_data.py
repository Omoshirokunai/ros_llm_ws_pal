#--- new
import os

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
            # Fetch the image from the remote server
            # sftp.get(REMOTE_IMAGE_PATH, LOCAL_IMAGE_PATH)
            sftp.get(remote_image_path, local_image_path)

        except Exception as e:
            print(local_image_path)
            print(f"Failed to fetch image: {e}")
        finally:
            sftp.close()
    except Exception as e:
        print(f"Failed to connect to SSH server: {e}")
    finally:
        ssh_client.close()

# Function to run the camera capture script via SSH
def run_camera_capture_script(ssh_client):
    stdin, stdout, stderr = ssh_client.exec_command(f"python2 {REMOTE_SCRIPT_PATH}")
    output = stdout.read().decode()
    errors = stderr.read().decode()

    if errors:
        print(f"Error running camera capture script: {errors}")
    else:
        print("Camera capture script ran successfully.")
fetch_image_via_ssh()


