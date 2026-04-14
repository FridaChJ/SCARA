# test_mqtt_feedback_keys.py
# - Publishes simulated motor angles to robot/feedback
# - Subscribes to robot/commands → prints EVERY command as it arrives
# - Polls keyboard at 20 Hz to catch short Xbox controller touches

import json
import time
import random
import paho.mqtt.client as mqtt
import keyboard  # pip install keyboard

BROKER_IP = "192.168.174.1"
PORT      = 1883
USERNAME  = "your_username"
PASSWORD  = "your_password"

TOPIC_FEEDBACK = "robot/feedback"
TOPIC_COMMANDS = "robot/commands"

PUBLISH_INTERVAL = 0.5   # send new angles every 0.5 s
POLL_INTERVAL    = 0.05  # check keyboard every 50 ms (20 Hz)

# =============================================================================
# KEY → GUI ACTION MAP  (must match KEY_MAP + KEY_GRIPPER in GUI.py)
# RT is now mapped to M
# =============================================================================
KEY_TO_ACTION = {
    "up"          : "UP_1         (Y button)",
    "down"        : "DOWN_1       (A button)",
    "right"       : "RIGHT        (B button)",
    "left"        : "LEFT         (X button)",
    "w"           : "UP_2         (POV up)",
    "s"           : "DOWN_2       (POV down)",
    "-"           : "ROTATE_LEFT  (Stick left)",
    "right shift" : "ROTATE_RIGHT (Stick right)",
    "m"           : "GRIPPER      (RT → M)",      # changed from g to m
}

# =============================================================================
# MQTT CALLBACKS
# on_message runs in the MQTT background thread and prints immediately —
# this is why every repeated hold command shows up, not just the first one.
# =============================================================================
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.subscribe(TOPIC_COMMANDS)
        print(f"[MQTT] Connected — subscribed to '{TOPIC_COMMANDS}'")
    else:
        print(f"[MQTT] Connection failed (rc={rc})")

def on_message(client, userdata, msg):
    """Print every single command the GUI publishes — including hold repeats."""
    try:
        payload = json.loads(msg.payload.decode())
        action  = payload.get("action", "?")
        ts      = time.strftime("%H:%M:%S")
        print(f"  [{ts}] [GUI → MQTT] {action}")
    except Exception as e:
        print(f"  [GUI → MQTT] raw: {msg.payload}  ({e})")

# =============================================================================
# MQTT SETUP
# =============================================================================
client = mqtt.Client()
client.username_pw_set(USERNAME, PASSWORD)
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER_IP, PORT, keepalive=60)
client.loop_start()

# =============================================================================
# MAIN LOOP
# =============================================================================
print("Starting tester — Ctrl+C to stop")
print(f"  Angles published every {PUBLISH_INTERVAL}s")
print(f"  Xbox keys polled every {int(POLL_INTERVAL*1000)}ms")
print("-" * 50)

last_publish = 0
last_pressed = set()

try:
    while True:
        now = time.time()

        # ── Publish simulated angles ──────────────────────────────────────
        if now - last_publish >= PUBLISH_INTERVAL:
            angles  = [round(random.uniform(0, 180), 2) for _ in range(4)]
            client.publish(TOPIC_FEEDBACK, json.dumps({"encoders": angles}))
            print(f"[FEEDBACK] {angles}")
            last_publish = now

        # ── Poll keyboard — only print on state change ────────────────────
        currently_pressed = set()
        for key_name, action in KEY_TO_ACTION.items():
            try:
                if keyboard.is_pressed(key_name):
                    currently_pressed.add((key_name, action))
            except ValueError:
                pass

        for key_name, action in currently_pressed - last_pressed:
            print(f"  [XBOX ▼] {action}")
        for key_name, action in last_pressed - currently_pressed:
            print(f"  [XBOX ▲] {action} released")

        last_pressed = currently_pressed
        time.sleep(POLL_INTERVAL)

except KeyboardInterrupt:
    print("\nStopping tester...")
finally:
    client.loop_stop()
    client.disconnect()