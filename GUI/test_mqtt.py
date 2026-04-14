from mqtt_client import MQTTClient  # or whatever your file is called
import time

# -------------------------------
# CALLBACK (simulate your backend)
# -------------------------------
def on_message(topic, payload):
    print(f"[TEST CALLBACK] Topic: {topic}")
    print(f"[TEST CALLBACK] Payload: {payload}")

# -------------------------------
# CREATE CLIENT
# -------------------------------
client = MQTTClient(
    broker_ip="192.168.174.1",
    port=1883,
    username="your_user",
    password="your_pass",
    on_message_callback=on_message
)

# -------------------------------
# CONNECT
# -------------------------------
client.connect()

# -------------------------------
# TEST LOOP
# -------------------------------
while True:
    time.sleep(5)

    # Send test command
    test_data = {
        "motor1": 100,
        "motor2": 50
    }

    print("[TEST] Publishing...")
    client.publish("robot/commands", test_data)