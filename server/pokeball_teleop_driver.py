#!/usr/bin/env python3
"""
Poké Ball Plus Standalone Teleoperation Driver
Connects to Poké Ball Plus (MAC: 58:2F:40:8D:50:71) over BLE.
Controls Auxiliary Pedestal Spinner (Motor 7) & LeSlider Gantry (Motor 8) via HTTP API.

Control Specification:
- Joystick Left/Right held FIRST + Button A (Stick Click) -> Nudge Gantry 500 ticks Left/Right over 2s.
- Joystick Left/Right held FIRST + Button B (Top Red) -> Step Pedestal Spinner to next lower/higher preset angle.
- Standalone Button A or B presses (without Joystick tilt) -> 0 Action.
"""

import asyncio
import time
import json
import logging
import urllib.request
import threading
from bleak import BleakClient
try:
    import zmq
except ImportError:
    zmq = None

MAC_ADDRESS = "58:2F:40:8D:50:71"
INPUT_UUID = "6675e16c-f36d-4567-bb55-6b51e27a23e6"
API_URL = "http://127.0.0.1:8085"
ZMQ_ROVER_HOST = "127.0.0.1"
ZMQ_ROVER_PORT = 5558

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')


class PokeballTeleopDriver:
    def __init__(self, mac_address=MAC_ADDRESS, api_url=API_URL, zmq_host=ZMQ_ROVER_HOST, zmq_port=ZMQ_ROVER_PORT):
        self.mac_address = mac_address
        self.api_url = api_url
        self.client = None

        # ZMQ Drive Socket Init
        self.zmq_host = zmq_host
        self.zmq_port = zmq_port
        self.zmq_ctx = zmq.Context() if zmq else None
        self.zmq_sock = None
        self._init_zmq_socket()

        # Pedestal Spinner 45-degree Presets (Clamped at safety bounds +/-165.0 deg)
        self.preset_angles = [-165.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0, 165.0]
        self.preset_index = 4  # Default index 4 = 0.0 deg (Forward)

        # Movement lock state (prevents request stacking for auxiliary gantry/pedestal)
        self.is_busy = False
        self.busy_until = 0.0

        # Input state tracking
        self.last_buttons = 0
        self.counter = 0
        self.last_drive_send_time = 0.0

    def _init_zmq_socket(self):
        if not self.zmq_ctx:
            logging.warning("⚠️ ZMQ module not installed; ZMQ drive streaming unavailable.")
            return
        try:
            self.zmq_sock = self.zmq_ctx.socket(zmq.REQ)
            self.zmq_sock.setsockopt(zmq.RCVTIMEO, 150) # 150ms timeout
            self.zmq_sock.setsockopt(zmq.LINGER, 0)
            self.zmq_sock.connect(f"tcp://{self.zmq_host}:{self.zmq_port}")
            logging.info("Connected ZMQ drive client to tcp://%s:%d", self.zmq_host, self.zmq_port)
        except Exception as e:
            logging.warning("Failed to connect ZMQ socket: %s", e)
            self.zmq_sock = None

    def send_drive_command(self, x, y):
        """Dispatches continuous ZMQ drive commands (x=steering, y=throttle) to rover_daemon."""
        if not self.zmq_sock:
            return
        now = time.time()
        if now - self.last_drive_send_time < 0.033: # Max ~30 Hz
            return
        self.last_drive_send_time = now

        def _work():
            try:
                self.zmq_sock.send_json({"x": x, "y": y, "source": "pokeball"})
                _ = self.zmq_sock.recv_json()
            except Exception:
                # Reconnect socket on timeout/error
                try:
                    self.zmq_sock.close()
                except Exception:
                    pass
                self._init_zmq_socket()

        threading.Thread(target=_work, daemon=True).start()

    def _send_aux_request(self, endpoint, payload, lock_duration=2.2):
        """Asynchronously dispatches HTTP requests to Gantry/Pedestal daemon on Port 8085."""
        now = time.time()
        if self.is_busy or now < self.busy_until:
            logging.info("⚠️ Aux Request Ignored: Motor is currently executing a move.")
            return

        self.is_busy = True
        self.busy_until = now + lock_duration

        def _work():
            try:
                url = f"{self.api_url}{endpoint}"
                data = json.dumps(payload).encode('utf-8')
                req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'})
                with urllib.request.urlopen(req, timeout=3.0) as resp:
                    pass
            except Exception as e:
                logging.warning("Aux API HTTP Dispatch error: %s", e)
            finally:
                time.sleep(lock_duration)
                self.is_busy = False

        threading.Thread(target=_work, daemon=True).start()

    def notification_handler(self, sender, data):
        self.counter += 1
        if len(data) < 5:
            return

        buttons = data[1]

        # 12-Bit Packed Joystick Decoding (X: steering, Y: throttle)
        raw_x_12 = data[2] | ((data[3] & 0x0F) << 8)
        raw_y_12 = (data[3] >> 4) | (data[4] << 4)
        
        x_offset = raw_x_12 - 2048
        y_offset = raw_y_12 - 2048
        
        norm_x = x_offset / 2048.0
        norm_y = y_offset / 2048.0
        
        # Deadzone filter
        if abs(norm_x) < 0.08:
            norm_x = 0.0
        if abs(norm_y) < 0.08:
            norm_y = 0.0
            
        # Stream drive commands over ZMQ to rover_daemon
        self.send_drive_command(norm_x, norm_y)
        
        now = time.time()

        # 1. Button B (Top Red Button): Requires Joystick Left/Right direction engaged FIRST
        if (buttons & 0x01) and not (self.last_buttons & 0x01):
            if self.is_busy or now < self.busy_until:
                logging.info("⚠️ Button B Ignored: Movement currently in progress.")
            else:
                if x_offset > 200:
                    self.preset_index = min(len(self.preset_angles) - 1, self.preset_index + 1)
                    target_deg = self.preset_angles[self.preset_index]
                    logging.info("Joystick Right -> Button B -> Stepping Pedestal Preset: %.1f deg (Index %d/%d)",
                                 target_deg, self.preset_index, len(self.preset_angles) - 1)
                    self._send_aux_request("/api/move", {"id": 7, "angle": target_deg}, lock_duration=2.2)
                elif x_offset < -200:
                    self.preset_index = max(0, self.preset_index - 1)
                    target_deg = self.preset_angles[self.preset_index]
                    logging.info("Joystick Left -> Button B -> Stepping Pedestal Preset: %.1f deg (Index %d/%d)",
                                 target_deg, self.preset_index, len(self.preset_angles) - 1)
                    self._send_aux_request("/api/move", {"id": 7, "angle": target_deg}, lock_duration=2.2)
                else:
                    logging.info("⚠️ Top Button B Press Ignored: Joystick direction was NOT engaged first.")

        # 2. Button A (Stick Click): Requires Joystick Left/Right direction engaged FIRST
        if (buttons & 0x02) and not (self.last_buttons & 0x02):
            if x_offset < -200:
                if self.is_busy or now < self.busy_until:
                    logging.info("⚠️ Gantry Move Ignored: Movement currently in progress.")
                else:
                    logging.info("Joystick Left -> Button A Clicked -> Nudging Gantry 500 ticks LEFT over 2s")
                    self._send_aux_request("/api/nudge_physical", {"id": 8, "direction": "left", "amount": 500}, lock_duration=2.2)
            elif x_offset > 200:
                if self.is_busy or now < self.busy_until:
                    logging.info("⚠️ Gantry Move Ignored: Movement currently in progress.")
                else:
                    logging.info("Joystick Right -> Button A Clicked -> Nudging Gantry 500 ticks RIGHT over 2s")
                    self._send_aux_request("/api/nudge_physical", {"id": 8, "direction": "right", "amount": 500}, lock_duration=2.2)
            else:
                logging.info("⚠️ Thumbstick A Click Ignored: Joystick direction was NOT engaged first.")

        self.last_buttons = buttons

    async def run(self):
        while True:
            try:
                logging.info("Connecting to Poké Ball Plus at %s...", self.mac_address)
                async with BleakClient(self.mac_address, timeout=6.0) as client:
                    self.client = client
                    logging.info("✅ Connected to Poké Ball Plus!")
                    await client.start_notify(INPUT_UUID, self.notification_handler)
                    logging.info("Listening for Poké Ball Plus telemetry...")
                    while client.is_connected:
                        await asyncio.sleep(1.0)
            except Exception as e:
                logging.info("Poké Ball BLE waiting for device (press button to wake up)... [%s]", e)
                await asyncio.sleep(3.0)


def main():
    driver = PokeballTeleopDriver()
    try:
        asyncio.run(driver.run())
    except KeyboardInterrupt:
        logging.info("Driver stopped by user.")


if __name__ == "__main__":
    main()
