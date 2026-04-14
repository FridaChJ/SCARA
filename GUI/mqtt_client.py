# =============================================================================
#BACKEND. MQTT Client
# =============================================================================
# Description : Class only handles MQTT, there is no processing of data.
#               - Connect to broker
#               - Handle authentication
#               - Auto-reconnect
#               - Subscribe to topics
#               - Publish info
#               - Call a callback when data arrives
# Authors     : Frida Sophia Chavez Juarez
# Menthor     : Oscar Vargas Perez
# Last updated: 4/3/2026. 9:36
# Dependencies: tkinter (stdlib)
# =============================================================================

import json
import time
import threading
import paho.mqtt.client as mqtt
from typing import Callable, Optional

class MQTTClient:
    def __init__(
        self,
        broker_ip: str,
        port: int,
        username: str,
        password: str,
        on_message_callback: Optional[Callable[[str, dict], None]] = None
    ):
        """
        MQTT Client wrapper.

        Args:
            broker_ip (str): IP address of the broker
            port (int): MQTT port (default 1883)
            username (str): MQTT username
            password (str): MQTT password
            on_message_callback (callable): function(topic: str, payload: dict)
        """
        self.broker_ip = broker_ip
        self.port = port
        self.username = username
        self.password = password
        self.on_message_callback: Optional[Callable[[str, dict], None]] = on_message_callback

        self.client = mqtt.Client()

        # Auth
        self.client.username_pw_set(username, password)

        # Callbacks
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect

        self._connected = False

    # ------------------------------------------------------------------
    # CONNECTION MANAGEMENT
    # ------------------------------------------------------------------

    def connect(self):
        """Start MQTT client in a separate thread."""
        thread = threading.Thread(target=self._run, daemon=True)
        thread.start()

    def _run(self):
        """Internal loop with auto-reconnect."""
        while True:
            try:
                print("[MQTT] Connecting...")
                self.client.connect(self.broker_ip, self.port, keepalive=60)
                self.client.loop_forever()
            except Exception as e:
                print(f"[MQTT] Connection failed: {e}")
                print("[MQTT] Reconnecting in 3 seconds...")
                time.sleep(3)

    # ------------------------------------------------------------------
    # CALLBACKS
    # ------------------------------------------------------------------

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("[MQTT] Connected successfully")
            self._connected = True

            # Subscribe to feedback topic
            client.subscribe("robot/feedback", qos=0)
            print("[MQTT] Subscribed to robot/feedback")

        else:
            print(f"[MQTT] Connection failed with code {rc}")

    def _on_disconnect(self, client, userdata, rc):
        print("[MQTT] Disconnected")
        self._connected = False

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            print(f"[MQTT] Received: {payload}")

            # Pass to controller
            if self.on_message_callback:
                self.on_message_callback(msg.topic, payload)

        except Exception as e:
            print(f"[MQTT] Error parsing message: {e}")

    # ------------------------------------------------------------------
    # PUBLISH
    # ------------------------------------------------------------------

    def publish(self, topic, data_dict):
        """Publish JSON data."""
        if not self._connected:
            print("[MQTT] Not connected, cannot publish")
            return

        try:
            payload = json.dumps(data_dict)
            self.client.publish(topic, payload, qos=0)
        except Exception as e:
            print(f"[MQTT] Publish error: {e}")