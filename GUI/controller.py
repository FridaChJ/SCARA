# =============================================================================
#BACKEND. Controller
# =============================================================================
# Description : Connects MQTT with GUI
#               - Makes sure info is in the needed MQTT format
#               - Manage feedback
#               - Update GUI
# Authors     : Frida Sophia Chavez Juarez
# Menthor     : Oscar Vargas Perez
# Last updated: 4/3/2026. 9:36
# Dependencies: tkinter (stdlib)
# =============================================================================
import queue
import collections
import json  
import time  # to store timestamps for dynamic plotting

MOTOR_NAMES = ["Base", "Height", "Elbow", "Gripper"]

class Controller:
    def __init__(self, mqtt_client, motor_vars, root):
        """
        Backend controller for GUI <-> MQTT communication.

        Args:
            mqtt_client : instance of MQTTClient
            motor_vars  : list of Tkinter StringVar (from GUI)
            root        : Tk root (needed for safe UI updates)
        """
        self.mqtt = mqtt_client
        self.motor_vars = motor_vars
        self.root = root
        
        # ── History buffers for plotting (sliding window) ──
        self.max_len = 200  # store last 100 samples
        self.history = {
            name: {"times": collections.deque(maxlen=self.max_len),
                    "values": collections.deque(maxlen=self.max_len)}
                    for name in MOTOR_NAMES 
}
        # Plot callback placeholder (GUI will set this)
        self.plot_update_callback = None
        # Thread-safe queue for GUI updates
        self.queue = queue.Queue()

        # Start GUI update loop
        self._start_gui_loop()

    # ------------------------------------------------------------------
    # GUI → MQTT
    # ------------------------------------------------------------------

    def send_command(self, action):
        """
        Forward GUI action directly to MQTT.
        The motor logic / calculations are handled elsewhere.
        """
        payload = {"action": action}
        self.mqtt.publish("robot/commands", payload)

    # ------------------------------------------------------------------
    # MQTT → CONTROLLER
    # ------------------------------------------------------------------

    def handle_mqtt_message(self, topic, payload):
        """
        Called from MQTT thread. Updates GUI with encoder feedback and history.
        """
        if topic == "robot/feedback":
            encoders = payload.get("encoders", [])

            # Push data to queue for GUI thread
            self.queue.put(encoders)

            # ── Update history buffers ──
            timestamp = time.time()  # current time in seconds

            for i, name in enumerate(MOTOR_NAMES):
                if i < len(encoders):
                    self.history[name]["times"].append(timestamp)
                    self.history[name]["values"].append(encoders[i])
    # ------------------------------------------------------------------
    # GUI UPDATE LOOP (SAFE)
    # ------------------------------------------------------------------

    def _start_gui_loop(self):
        """Continuously process queue in main thread."""
        self._process_queue()

    def _process_queue(self):
        while not self.queue.empty():
            encoders = self.queue.get()

            # Update GUI safely
            for i, value in enumerate(encoders):
                if i < len(self.motor_vars):
                    self.motor_vars[i].set(f"{value:.2f}°")

        # Trigger plot update in GUI thread (if set)
        if self.plot_update_callback:
            self.root.after(0, self.plot_update_callback)

        # Schedule next check
        self.root.after(50, self._process_queue)  # ~20 FPS