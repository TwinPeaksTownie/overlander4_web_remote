import os
import sys
import time
import json
import logging
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import parse_qs, urlparse

from aux_servo_controller import AuxiliaryServoController

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CALIB_FILE = os.path.join(SCRIPT_DIR, "calibration_aux.json")
STATE_FILE = os.path.join(SCRIPT_DIR, "gantry_state.json")

def ticks_to_degrees_s7(ticks: int) -> float:
    """Converts raw Motor 7 ticks (0-4095) to intuitive degrees where 0 deg is Forward (tick 2048)."""
    deg = (int(ticks) - 2048) * 360.0 / 4096.0
    while deg > 180.0:
        deg -= 360.0
    while deg <= -180.0:
        deg += 360.0
    return round(deg, 1)

def degrees_to_ticks_s7(deg: float) -> int:
    """Converts intuitive degrees (0 deg Forward, -90 deg Left, +90 deg Right) to raw Motor 7 ticks (0-4095).
    Enforces a strict safety limit of [-165.0, +165.0] degrees under all circumstances.
    """
    clamped_deg = max(-165.0, min(165.0, float(deg)))
    ticks = int(round(2048 + (clamped_deg * 4096.0 / 360.0)))
    return ticks


class HardwareState:
    def __init__(self):
        self.lock = threading.Lock()
        self.ctrl = None
        self.raw_positions = {7: 2048, 8: 2503}
        self.accumulated_positions = {7: 2048, 8: 2503}
        self.last_raw_positions = {7: None, 8: None}
        self.torque_state = {7: False, 8: False}
        self.is_moving = {7: False, 8: False}
        self.calibration = {
            "7": {"min": 0, "max": 4095, "home": 2048, "home_deg": 0.0},
            "8": {}
        }
        self.hardware_active = False
        self.active_port = "None"
        self.error_msg = None

    def load_calibration(self):
        if os.path.exists(CALIB_FILE):
            try:
                with open(CALIB_FILE, "r") as f:
                    data = json.load(f)
                    self.calibration.update(data)
                logging.info(f"Loaded calibration from {CALIB_FILE}: {self.calibration}")
            except Exception as e:
                logging.error(f"Failed to load calibration file: {e}")
        self.load_state()

    def save_calibration(self):
        try:
            with open(CALIB_FILE, "w") as f:
                json.dump(self.calibration, f, indent=2)
            logging.info(f"Saved calibration to {CALIB_FILE}")
            return True
        except Exception as e:
            logging.error(f"Failed to save calibration: {e}")
            return False

    def load_state(self):
        if os.path.exists(STATE_FILE):
            try:
                with open(STATE_FILE, "r") as f:
                    data = json.load(f)
                    for sid_str, pos in data.items():
                        sid = int(sid_str)
                        pos_val = int(pos)
                        self.accumulated_positions[sid] = pos_val
                        if sid == 8:
                            self.raw_positions[sid] = pos_val
                        else:
                            self.raw_positions[sid] = pos_val % 4096
                logging.info(f"Loaded persistent state from {STATE_FILE}: {data}")
            except Exception as e:
                logging.error(f"Failed to load state file: {e}")

    def save_state(self):
        try:
            with open(STATE_FILE, "w") as f:
                json.dump({"7": self.accumulated_positions.get(7, 2048), "8": self.accumulated_positions.get(8, 2500)}, f, indent=2)
        except Exception as e:
            logging.error(f"Failed to save state: {e}")

    def update_raw_pos(self, sid: int, raw_pos: int):
        raw_pos = int(raw_pos)
        if sid == 8:
            # Native Mode 3 signed multi-turn
            self.raw_positions[sid] = raw_pos
            self.accumulated_positions[sid] = raw_pos
            return raw_pos

        # Servo 7 is a single-turn joint bounded to [-165, 165] deg.
        # Direct raw assignment without delta accumulation to prevent drift.
        raw_pos = raw_pos % 4096
        self.raw_positions[sid] = raw_pos
        self.accumulated_positions[sid] = raw_pos
        return raw_pos

    def move_target(self, sid: int, target_acc: int, step_size: int = 50, speed: int = 400, max_t: int = 1000):
        with self.lock:
            if not self.ctrl or not self.hardware_active:
                return False, "Hardware offline"
            
            if self.is_moving.get(sid, False):
                logging.warning(f"Rejected move for Servo {sid}: Motor is currently executing a move.")
                return False, "Motor is currently executing a move. Request ignored."

            self.is_moving[sid] = True
            try:
                # Auto-enable torque state on move command
                self.torque_state[sid] = True
                self.ctrl.set_torque(sid, True, max_torque_enable=max_t)

                if sid == 7:
                    deg = ticks_to_degrees_s7(target_acc)
                    clamped_deg = max(-165.0, min(165.0, deg))
                    target_acc = degrees_to_ticks_s7(clamped_deg)
                    self.ctrl.write_goal_raw(7, target_acc, speed=speed)
                    self.accumulated_positions[7] = target_acc
                    self.raw_positions[7] = target_acc
                elif sid == 8:
                    # Hardcoded physical safety bounds [3, 4800] - Absolutely no moves permitted beyond 4800
                    target_acc = max(3, min(4800, int(target_acc)))
                    curr = self.accumulated_positions.get(8, 2503)
                    delta = target_acc - curr
                    self.ctrl.set_position_multiturn(8, delta, speed=speed)
                    self.accumulated_positions[8] = target_acc
                    self.raw_positions[8] = target_acc

                if sid == 7:
                    final_raw = self.ctrl.read_pos(sid)
                    if final_raw is not None:
                        self.update_raw_pos(sid, final_raw)
                
                self.save_state()
                return True, "OK"
            finally:
                self.is_moving[sid] = False

    def start_background_polling(self):
        def _poll_worker():
            while True:
                time.sleep(0.1)
                if not self.hardware_active:
                    continue
                with self.lock:
                    if self.ctrl:
                        for sid in [7, 8]:
                            # Suspend polling if motor is actively executing a move command to prevent bus contention
                            if self.is_moving.get(sid, False):
                                continue
                            try:
                                if sid == 8:
                                    # Skip polling Reg 56 for Servo 8 (Mode 3).
                                    continue
                                else:
                                    pos = self.ctrl.read_pos(sid)
                                
                                if pos is not None:
                                    self.update_raw_pos(sid, pos)
                                else:
                                    logging.debug(f"Bus collision: None returned for servo {sid}. Keeping last known pos.")
                            except Exception as e:
                                logging.debug(f"Bus collision on servo {sid}: {e}. Keeping last known pos.")

        t = threading.Thread(target=_poll_worker, daemon=True)
        t.start()

hw = HardwareState()
hw.load_calibration()

class WebRequestHandler(BaseHTTPRequestHandler):
    def _send_json(self, data, code=200):
        body = json.dumps(data).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, filepath, mime="text/html"):
        if os.path.exists(filepath):
            with open(filepath, "rb") as f:
                content = f.read()
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(content)
        else:
            self.send_response(404)
            self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path in ["/", "/index.html"]:
            static_file = os.path.join(os.path.dirname(__file__), "static", "gantry_ui.html")
            self._send_file(static_file, "text/html")
        elif parsed.path == "/api/status":
            with hw.lock:
                left_b = hw.calibration.get("8", {}).get("left")
                right_b = hw.calibration.get("8", {}).get("right")
                curr_acc = hw.accumulated_positions.get(8, 2503)
                
                if left_b is not None and right_b is not None and left_b != right_b:
                    span = max(1, abs(right_b - left_b))
                    min_b = min(left_b, right_b)
                    pct = round(max(0.0, min(100.0, ((curr_acc - min_b) / span) * 100.0)), 1)
                else:
                    pct = None

                resp = {
                    "daemon": "aux_daemon",
                    "status": "online" if hw.hardware_active else "offline",
                    "hardware_connected": hw.hardware_active,
                    "active_port": hw.active_port,
                    "error": hw.error_msg,
                    "servos": {
                        "7": {
                            "pos": hw.accumulated_positions.get(7, 2048),
                            "raw": hw.raw_positions.get(7, 2048),
                            "angle": ticks_to_degrees_s7(hw.accumulated_positions.get(7, 2048)),
                            "torque": hw.torque_state.get(7, False),
                            "is_moving": hw.is_moving.get(7, False)
                        },
                        "8": {
                            "pos": hw.accumulated_positions.get(8, 2503),
                            "raw": hw.raw_positions.get(8, 2503),
                            "pct": pct,
                            "torque": hw.torque_state.get(8, False),
                            "is_moving": hw.is_moving.get(8, False)
                        }
                    },
                    "calibration": hw.calibration
                }
            self._send_json(resp)
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        content_length = int(self.headers.get('Content-Length', 0))
        body_bytes = self.rfile.read(content_length)
        body = json.loads(body_bytes.decode('utf-8')) if body_bytes else {}
        parsed = urlparse(self.path)

        if parsed.path == "/api/move":
            sid = int(body.get("id", 7))
            if "angle" in body and sid == 7:
                target_acc = degrees_to_ticks_s7(float(body["angle"]))
            else:
                target_acc = int(body.get("target", 2048))
            max_torque_req = 800 if sid == 7 else 500
            ok, msg = hw.move_target(sid, target_acc, step_size=50, speed=400, max_t=max_torque_req)
            if not ok:
                self._send_json({"status": "error", "message": msg}, 400)
            else:
                self._send_json({"status": "ok", "id": sid, "target_acc": target_acc, "angle": ticks_to_degrees_s7(target_acc) if sid == 7 else None})

        elif parsed.path == "/api/nudge_physical":
            sid = int(body.get("id", 8))
            direction = str(body.get("direction", "right")).lower()
            amount = abs(int(body.get("amount", 100)))

            with hw.lock:
                calib = hw.calibration.get(str(sid), {})
                left_b = calib.get("left")
                right_b = calib.get("right")
                
                is_inverted = (left_b > right_b) if (left_b is not None and right_b is not None) else False

                if direction == "left":
                    delta = amount if is_inverted else -amount
                else:
                    delta = -amount if is_inverted else amount

                curr_acc = hw.accumulated_positions.get(sid, 2503 if sid == 8 else 2048)
                target_acc = curr_acc + delta

                if left_b is not None and right_b is not None and not body.get("force", False):
                    true_min = min(left_b, right_b)
                    true_max = max(left_b, right_b)
                    target_acc = max(true_min, min(true_max, target_acc))

            # Move using appropriate hardware mechanism with 50% torque cap for over-current protection
            ok, msg = hw.move_target(sid, target_acc, step_size=50, speed=400, max_t=500)
            if not ok:
                self._send_json({"status": "error", "message": msg}, 400)
            else:
                logging.info(f"Aux Daemon Nudge Physical {direction.upper()}: Servo {sid} -> target_acc {target_acc}")
                self._send_json({"status": "ok", "id": sid, "direction": direction, "target_acc": target_acc, "current_acc": hw.accumulated_positions.get(sid)})

        elif parsed.path == "/api/torque":
            sid = int(body.get("id", 7))
            toggle = body.get("toggle", True)
            with hw.lock:
                new_state = not hw.torque_state[sid] if toggle else bool(body.get("enable", False))
                hw.torque_state[sid] = new_state
                if hw.hardware_active and hw.ctrl:
                    hw.ctrl.set_torque(sid, new_state)
            self._send_json({"status": "ok", "id": sid, "torque": new_state})

        elif parsed.path == "/api/sync_position":
            sid = int(body.get("id", 8))
            pos = int(body.get("pos", -3))
            with hw.lock:
                hw.accumulated_positions[sid] = pos
                hw.raw_positions[sid] = pos
                hw.save_state()
            self._send_json({"status": "ok", "id": sid, "synced_pos": pos})

        elif parsed.path == "/api/calibration":
            with hw.lock:
                if "calibration" in body:
                    hw.calibration.update(body["calibration"])
                    hw.save_calibration()
            self._send_json({"status": "ok", "calibration": hw.calibration})
        else:
            self._send_json({"error": "Endpoint not found"}, 404)


def connect_hardware():
    try:
        ctrl = AuxiliaryServoController(port='/dev/ttyACM0', baudrate=1000000)
        hw.ctrl = ctrl
        hw.hardware_active = True
        hw.active_port = '/dev/ttyACM0'
        
        # Force Torque OFF on startup for safety
        ctrl.set_torque(7, False)
        ctrl.set_torque(8, False)
        hw.torque_state[7] = False
        hw.torque_state[8] = False
        
        # Configure Servo 8 for Mode 3 (Multi-Turn)
        ctrl.setup_multi_turn_mode(8)
        
        # Read initial pos without writing EEPROM
        pos7 = ctrl.read_pos(7)
        pos8 = ctrl.get_position_multiturn(8)
        if pos7 is not None: hw.update_raw_pos(7, pos7)
        
        hw.start_background_polling()
        logging.info("Aux Daemon connected hardware on /dev/ttyACM0 (Torque OFF on startup)")
    except Exception as e:
        logging.error(f"Hardware connection failed: {e}")
        hw.hardware_active = False

def main():
    connect_hardware()
    server = HTTPServer(('0.0.0.0', 8085), WebRequestHandler)
    logging.info("Aux Daemon running on http://0.0.0.0:8085")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
