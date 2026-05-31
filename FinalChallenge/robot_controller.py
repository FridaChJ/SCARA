# =============================================================================
# MAIN CONTROLLER — Sequential Robot Movement
# =============================================================================
# Description : Sends joint angles one at a time via MQTT and waits for a
#               "Finished" confirmation before proceeding to the next object.
#               Supports testing mode where "Finished" can be typed manually.
# Authors     : (your name)
# Mentor      : Oscar Vargas Perez
# Last updated: 2026
# Dependencies: mqtt_client.py, Kinematics.py, paho-mqtt
# =============================================================================

import threading
from typing import Optional
from mqtt_client import MQTTClient

# ── MQTT Topics ───────────────────────────────────────────────────────────────
TOPIC_COMMAND = "angles/command"
TOPIC_STATUS  = "system/status"


class RobotController:
    def __init__(
        self,
        mqtt_client: MQTTClient,
        testing_mode: bool = False,
    ):
        """
        Sequential robot movement controller.

        Parameters
        ----------
        mqtt_client  : Connected MQTTClient instance.
        testing_mode : If True, also listens for a manual "Finished" typed in
                       the terminal (useful for development without hardware).
        """
        self.mqtt       = mqtt_client
        self.testing    = testing_mode

        # Shared flag — set to True when "Finished" arrives (MQTT or terminal)
        self._finished_event = threading.Event()

        # Register our status handler on top of any existing callback
        self._prev_callback = mqtt_client.on_message_callback
        self.mqtt.on_message_callback = self._on_message

        # Subscribe to the completion topic
        self.mqtt.subscribe(TOPIC_STATUS, qos=0)

    # ── Internal MQTT callback ────────────────────────────────────────────────

    def _on_message(self, topic: str, payload: dict):
        """Called by MQTTClient whenever a message arrives on any subscribed topic."""

        # Let any pre-existing callback run first
        if self._prev_callback:
            self._prev_callback(topic, payload)

        if topic == TOPIC_STATUS:
            # Payload can be {"status": "Finished"} or just the string "Finished"
            status = (
                payload.get("status", "") if isinstance(payload, dict)
                else str(payload)
            )
            if status.strip() == "Finished":
                print("[CTRL] 'Finished' received via MQTT — proceeding.")
                self._finished_event.set()

    # ── Wait helpers ─────────────────────────────────────────────────────────

    def _wait_for_finished(self):
        """
        Block until "Finished" arrives.

        In testing mode a background thread also accepts the string typed in
        the terminal so you can simulate the signal without real hardware.
        """
        self._finished_event.clear()

        if self.testing:
            # Launch a daemon thread that reads stdin
            t = threading.Thread(target=self._read_terminal, daemon=True)
            t.start()

        # Block here — released by _on_message OR _read_terminal
        self._finished_event.wait()

    def _read_terminal(self):
        """Simulate the 'Finished' signal by typing it in the terminal."""
        while not self._finished_event.is_set():
            try:
                line = input("[TEST ] Type 'Finished' to simulate confirmation: ")
            except EOFError:
                break
            if line.strip() == "Finished":
                print("[CTRL] 'Finished' received via terminal — proceeding.")
                self._finished_event.set()
                break

    # ── Format helper ─────────────────────────────────────────────────────────

    @staticmethod
    def _format_command(name: str, j1: float, j3: float) -> str:
        """
        Build the strict MQTT command string.

        Format: <name>,j1:<angle>,j3:<angle>
        Example: red1,j1:34.2,j3:120.5
        """
        return f"{name},j1:{j1},j3:{j3}"

    # ── Main execution loop ───────────────────────────────────────────────────

    def run(self, angles: dict[str, Optional[dict[str, float]]]):
        """
        Execute the full movement sequence.

        Parameters
        ----------
        angles : dict from compute_kinematics()
                 { "red1": {"j1": 34.2, "j3": 120.5}, "red2": None, ... }
        """
        # Filter out unreachable objects while preserving insertion order
        valid = [
            (name, vals)
            for name, vals in angles.items()
            if vals is not None
        ]

        total = len(valid)
        if total == 0:
            print("[CTRL] No valid targets to execute.")
            return

        print(f"\n[CTRL] Starting sequence — {total} cycle(s) to execute.\n")

        for cycle_num, (name, vals) in enumerate(valid, start=1):
            j1 = vals["j1"]
            j3 = vals["j3"]

            command = self._format_command(name, j1, j3)
            print(f"[CTRL] Cycle {cycle_num}/{total} — Sending: {command}")

            # Publish to MQTT broker
            self.mqtt.publish(TOPIC_COMMAND, {"command": command})

            # Block until hardware (or tester) signals completion
            self._wait_for_finished()
            print(f"[CTRL] Cycle {cycle_num}/{total} complete.\n")

        print("[CTRL] All cycles finished. Sequence complete.")


# =============================================================================
# Entry point (demo / testing)
# =============================================================================
if __name__ == "__main__":
    from Kinematics import compute_kinematics # type: ignore

    # ── Robot & MQTT configuration ────────────────────────────────────────────
    BROKER_IP = "192.168.1.100"   # <-- replace with your broker IP
    PORT      = 1883
    USERNAME  = "robot"           # <-- replace
    PASSWORD  = "secret"          # <-- replace

    L1 = 25.0   # cm
    L2 = 20.0   # cm

    # ── Sample vision data ────────────────────────────────────────────────────
    coords_data = [
        {"label": "red",   "x": 33.24, "y": 16.35},
        {"label": "red",   "x": 13.74, "y": 16.10},
        {"label": "blue",  "x": 25.23, "y": 25.40},
        {"label": "blue",  "x": 25.34, "y": 20.49},
        {"label": "blue",  "x": 25.40, "y":  8.80},
        {"label": "green", "x": 25.32, "y": 30.77},
        {"label": "green", "x": 25.26, "y": 15.45},
        {"label": "green", "x": 25.54, "y":  3.32},
    ]

    # ── Compute IK ────────────────────────────────────────────────────────────
    angles = compute_kinematics(coords_data, l1=L1, l2=L2, elbow_up=False)

    print(f"{'Object':<10} {'J1 (°)':>10} {'J3 (°)':>10}")
    print("-" * 32)
    for name, vals in angles.items():
        if vals is None:
            print(f"{name:<10} {'UNREACHABLE':>21}")
        else:
            print(f"{name:<10} {vals['j1']:>10.4f} {vals['j3']:>10.4f}")

    # ── Connect MQTT ──────────────────────────────────────────────────────────
    client = MQTTClient(
        broker_ip=BROKER_IP,
        port=PORT,
        username=USERNAME,
        password=PASSWORD,
    )
    client.connect()

    import time
    time.sleep(1.5)     # allow connection to establish

    # ── Run controller ────────────────────────────────────────────────────────
    # Set testing_mode=True to confirm each step by typing "Finished" in terminal
    controller = RobotController(client, testing_mode=True)
    controller.run(angles)