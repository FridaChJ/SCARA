
# ===========================IMPORTS===========================================
#MQTT  
import time
from mqtt_client import MQTTClient
#Camera
import requests
import os
#GUI
import subprocess
#CV
from CV import process_image
#Kinematics
from Kinematics import compute_kinematics # pyright: ignore[reportMissingImports]
#Controller
from robot_controller import RobotController   # type: ignore
# ===========================CONSTANTS=======================================
# MQTT
BROKER_IP = "192.168.4.76"
PORT = 1883
USERNAME = "your_user"
PASSWORD = "your_password"
TOPICS = ["angles/response", "system/status"]
#Camera
IP_WEBCAM_URL = "http://192.168.4.79:8080/shot.jpg"
SAVE_DIR = r"D:\Reto\FinalChallenge"
IMAGE_PATH = os.path.join(SAVE_DIR, "foto.jpg")
#GUI
GUI_DIR = r"D:\Reto\GUI"
GUI_SCRIPT = "GUI.py"
#Kinematics
L1 = 25.0  # cm
L2 = 20.0  # cm
ELBOW_UP = False
## ===========================Retry=======================================
#Retry
import time

def run_with_retry(task_fn, task_name="Task", max_retries=10, delay=2):
    """
    Runs any function with retry logic.

    Args:
        task_fn (function): function that returns True (success) or False (fail)
        task_name (str): name for logging
        max_retries (int): number of attempts
        delay (int): seconds between retries
    """

    for attempt in range(max_retries):
        try:
            print(f"[MAIN] {task_name} ({attempt+1}/{max_retries})...")

            result = task_fn()

            if result:
                print(f"[MAIN] {task_name} successful")
                return True
            else:
                raise Exception("Returned False")

        except Exception as e:
            print(f"[MAIN] ERROR: {task_name} failed -> {e}")
            time.sleep(delay)

    print(f"[MAIN] ERROR: {task_name} failed after {max_retries} attempts")
    return False
def run_cv_until_valid(max_attempts=10, delay=2):
    """
    Runs CV repeatedly until:
    - ArUco markers are detected (no exception)
    
    If it fails:
    - capture a new image
    - retry

    Returns:
        summary_data, coordinates_data
    """

    for attempt in range(max_attempts):
        try:
            print(f"[MAIN] CV Processing ({attempt+1}/{max_attempts})...")

            summary_data, coordinates_data = process_image(IMAGE_PATH) # type: ignore

            # If process_image did NOT raise exception → markers detected
            print("[MAIN] CV successful")
            return summary_data, coordinates_data

        except Exception as e:
            print(f"[MAIN] CV ERROR: {e}")

            # Take a new photo before retrying
            print("[MAIN] Retaking image...")
            if not capture_image():
                print("[MAIN] Camera capture failed during retry")

            time.sleep(delay)

    print("[MAIN] ERROR: CV failed after max attempts")
    return None, None
# ===========================MQTT CONNECTION=======================================
#Callback function for MQTT messages
def on_message(topic, payload):
    print(f"[MAIN] Received on {topic}: {payload}")

def init_mqtt():
    global mqtt

    mqtt = MQTTClient(
        BROKER_IP,
        PORT,
        USERNAME,
        PASSWORD,
    )

    mqtt.connect()

    time.sleep(2)

    return mqtt._connected

# ===========================CAMERA CONNECTION=======================================
def capture_image():
    response = requests.get(IP_WEBCAM_URL, timeout=5)

    if response.status_code == 200:
        with open(IMAGE_PATH, "wb") as f:
            f.write(response.content)
        return True
    else:
        return False
    
# ===========================GUI CONNECTION=======================================
def launch_gui():
    try:
        subprocess.Popen(
            ["python", GUI_SCRIPT],
            cwd=GUI_DIR
        )
        return True
    except Exception as e:
        print(f"[MAIN] GUI launch error: {e}")
        return False

# ===========================Kinematics=======================================
def run_kinematics(coordinates_data):
    """
    Runs inverse kinematics on CV output
    """

    try:
        print("[MAIN] Running kinematics...")

        angles = compute_kinematics(
            coordinates_data,
            l1=L1,
            l2=L2,
            elbow_up=ELBOW_UP
        )

        print("[MAIN] Kinematics successful")
        return angles

    except Exception as e:
        print(f"[MAIN] Kinematics ERROR: {e}")
        return None
#====================================================================================
# ===========================MAIN=======================================
#====================================================================================
if __name__ == "__main__":
    # Initialize MQTT with retry
    if not run_with_retry(init_mqtt, "MQTT Connection"):
        print("[MAIN] Exiting due to MQTT connection failure")
        exit(1)

    # Capture image with retry
    if not run_with_retry(capture_image, "Camera Capture"):
        print("[MAIN] Exiting due to camera capture failure")
        exit(1)
    
    # Launch GUI with retry
    if not run_with_retry(launch_gui, "GUI Launch"):
        print("[MAIN] Exiting due to GUI launch failure")
        exit(1)

    # Run CV with custom retry
    summary_data, coordinates_data = run_cv_until_valid()

    if summary_data is None:
        print("[MAIN] Exiting due to CV failure")
        exit(1)

    print("[MAIN] CV Output:")
    print(summary_data)
    print(coordinates_data)

    #Run Kinematics
    angles = run_kinematics(coordinates_data)

    if angles is None:
        print("[MAIN] Exiting due to kinematics failure")
        exit(1)

    print("[MAIN] Kinematics Output:")
    print(angles)

    #Controller
    print("[MAIN] Starting movement sequence...")
    controller = RobotController(mqtt, testing_mode=True)  # testing_mode=False for production
    controller.run(angles)

    print("[MAIN] System complete.")