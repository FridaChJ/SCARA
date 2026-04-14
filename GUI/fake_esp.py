import paho.mqtt.client as mqtt
import json
import time

client = mqtt.Client()
client.username_pw_set("your_user", "your_pass")
client.connect("192.168.174.1", 1883, 60)

while True:
    data = {
        "encoder1": 111,
        "encoder2": 222
    }

    client.publish("robot/feedback", json.dumps(data))
    print("[FAKE ESP] Sent feedback")

    time.sleep(2)