import http.server
import socketserver
import json
import urllib.parse
import threading
import time
import serial
import os
import cv2
import numpy as np
import requests
try:
    import zmq
except ImportError:
    zmq = None

PORT = 8080

# Hardware UART Serial Port on Raspberry Pi 500 / Pi 4B (prioritize /dev/serial0 -> /dev/ttyAMA10)
SERIAL_PORT = "/dev/serial0" if os.path.exists("/dev/serial0") else ("/dev/ttyAMA10" if os.path.exists("/dev/ttyAMA10") else ("/dev/ttyAMA0" if os.path.exists("/dev/ttyAMA0") else "/dev/ttyACM0"))

def detect_follower_port():
    import os
    import glob
    ports = glob.glob("/dev/serial/by-id/*")
    for p in ports:
        if "Adafruit_KB2040" in p or "KB2040" in p:
            continue
        if "5AAF262436" in p:  # Reachy Mini serial
            continue
        return p
    # Fallback to listing ttyACM/ttyUSB
    for dev in ["/dev/ttyACM2", "/dev/ttyUSB0", "/dev/ttyACM1", "/dev/ttyACM0"]:
        if os.path.exists(dev):
            try:
                real_kb = os.path.realpath(SERIAL_PORT)
            except Exception:
                real_kb = ""
            try:
                real_reachy = os.path.realpath("/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AAF262436-if00")
            except Exception:
                real_reachy = ""
            real_dev = os.path.realpath(dev)
            if real_dev != real_kb and real_dev != real_reachy:
                return dev
    return "/dev/ttyACM2"

# Global state
lock = threading.Lock()
latest_telemetry = {
    "mode": "WEB",
    "left_out": 1500,
    "right_out": 1500,
    "sbus_active": 0,
    "web_active": 0,
    "ch1": 1000,
    "ch2": 1000,
    "ch5": 1000,
    "ch6": 1000,
    "slider_pos": 1500,
    "distance": 0,
    "flags": 0,
    "so_arm_active": 2, # 0: inactive, 1: active, 2: offline
    "reachy_daemon_active": 2, # 0: inactive, 1: active, 2: offline
    "tts_mover_active": 2,
    "hot_mic_active": 2
}

# Joystick target values (Y = throttle, X = steering)
joystick_x = 0.0
joystick_y = 0.0
last_joystick_update = 0.0
macro_active = False
lights_pulse_trigger = 0

# Camera state for dual cameras (0: USB Driving Cam, 2: Reachy head cam)
latest_frames = {0: None, 2: None}
camera_locks = {0: threading.Lock(), 2: threading.Lock()}

def find_camera_index_by_name(name_substring):
    import os
    try:
        for i in range(40):
            name_file = f"/sys/class/video4linux/video{i}/name"
            if os.path.exists(name_file):
                with open(name_file, "r") as f:
                    name = f.read().strip()
                if name_substring.lower() in name.lower():
                    return i
    except Exception:
        pass
    return None

# Camera capture worker
def camera_worker(device_idx):
    global latest_frames
    name = "DRIVING CAM" if device_idx == 0 else "REACHY CAM"
    print(f"Starting camera worker thread for /dev/video{device_idx} ({name})...")
    
    if device_idx == 2:
        # Fetch from local Reachy Mini daemon API on port 8000 (since it holds the device open)
        url = "http://127.0.0.1:8000/api/camera/frame"
        while True:
            try:
                r = requests.get(url, timeout=1.0)
                if r.status_code == 200:
                    with camera_locks[2]:
                        latest_frames[2] = r.content
                else:
                    # Offline placeholder
                    placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.rectangle(placeholder, (15, 15), (625, 465), (20, 20, 30), -1)
                    cv2.rectangle(placeholder, (10, 10), (630, 470), (0, 70, 255), 1)
                    cv2.putText(placeholder, "REACHY CAM OFFLINE", (170, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 70, 255), 2)
                    _, jpeg = cv2.imencode('.jpg', placeholder)
                    with camera_locks[2]:
                        latest_frames[2] = jpeg.tobytes()
                    time.sleep(1.0)
                time.sleep(0.04) # ~25 FPS
            except Exception:
                placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.rectangle(placeholder, (15, 15), (625, 465), (20, 20, 30), -1)
                cv2.rectangle(placeholder, (10, 10), (630, 470), (0, 70, 255), 1)
                cv2.putText(placeholder, "REACHY CAM OFFLINE", (170, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 70, 255), 2)
                _, jpeg = cv2.imencode('.jpg', placeholder)
                with camera_locks[2]:
                    latest_frames[2] = jpeg.tobytes()
                time.sleep(2.0)
    else:
        # Standard OpenCV capture for USB webcam (Wed Camera)
        cap = None
        last_index = None
        while True:
            try:
                idx = find_camera_index_by_name("Wed Camera")
                if idx is None:
                    if cap is not None:
                        cap.release()
                        cap = None
                    last_index = None
                    
                    placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.rectangle(placeholder, (15, 15), (625, 465), (20, 20, 30), -1)
                    cv2.rectangle(placeholder, (10, 10), (630, 470), (0, 70, 255), 1)
                    cv2.putText(placeholder, "DRIVING CAM OFFLINE", (170, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 70, 255), 2)
                    _, jpeg = cv2.imencode('.jpg', placeholder)
                    with camera_locks[0]:
                        latest_frames[0] = jpeg.tobytes()
                    time.sleep(1.0)
                    continue
                
                if cap is None or not cap.isOpened() or idx != last_index:
                    if cap is not None:
                        cap.release()
                    cap = cv2.VideoCapture(idx)
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    last_index = idx
                    time.sleep(1.0)
                    
                if cap.isOpened():
                    success, frame = cap.read()
                    if success:
                        # Compress to JPEG
                        _, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                        frame_bytes = jpeg.tobytes()
                        with camera_locks[0]:
                            latest_frames[0] = frame_bytes
                    else:
                        print(f"Failed to read camera {name}. Re-initializing...")
                        cap.release()
                        cap = None
                else:
                    time.sleep(1.0)
                    
                time.sleep(0.04) # ~25 FPS
                
            except Exception as e:
                print(f"Camera worker error for {name}: {e}")
                if cap:
                    cap.release()
                    cap = None
                time.sleep(2.0)

# SO-101 arm status poller thread (runs pgrep locally)
def so_arm_poller():
    global latest_telemetry
    import subprocess
    print("Starting SO-101 arm status poller thread...")
    while True:
        try:
            res = subprocess.run(
                ["pgrep", "-f", "sewer_daemon.py"],
                capture_output=True, text=True, timeout=3.0
            )
            if res.returncode == 0:
                status = 1
            elif res.returncode == 1:
                status = 0
            else:
                status = 2
        except Exception:
            status = 2
            
        with lock:
            latest_telemetry["so_arm_active"] = status
            
        time.sleep(2.5)

# Reachy Mini daemon status poller thread (runs pgrep locally)
def reachy_daemon_poller():
    global latest_telemetry
    import subprocess
    import socket
    print("Starting Reachy daemon status poller thread...")
    while True:
        status = 2
        app_status = 2
        try:
            res = subprocess.run(
                ["pgrep", "-f", "reachy-mini-daemon"],
                capture_output=True, text=True, timeout=3.0
            )
            if res.returncode == 0:
                status = 1
                # Check if port 8042 is open
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(0.5)
                try:
                    s.connect(("127.0.0.1", 8042))
                    app_status = 1
                    s.close()
                except Exception:
                    app_status = 0
            elif res.returncode == 1:
                status = 0
                app_status = 0
        except Exception:
            pass
            
        with lock:
            latest_telemetry["reachy_daemon_active"] = status
            latest_telemetry["tts_mover_active"] = app_status
            
        time.sleep(2.5)

# Hot Mic status poller thread
def hot_mic_poller():
    global latest_telemetry
    import subprocess
    print("Starting Hot Mic status poller thread...")
    while True:
        status = 2
        try:
            res = subprocess.run(
                ["pgrep", "-f", "reachy_hot_mic.py"],
                capture_output=True, text=True, timeout=3.0
            )
            if res.returncode == 0:
                status = 1
            elif res.returncode == 1:
                status = 0
        except Exception:
            pass
            
        with lock:
            latest_telemetry["hot_mic_active"] = status
            
        time.sleep(2.5)

# ZMQ Drive Listener Thread on Port 5558 (Poké Ball & AI Agent Drive Interface)
def zmq_drive_worker():
    global joystick_x, joystick_y, last_joystick_update
    print("Starting ZMQ Drive Listener thread on tcp://0.0.0.0:5558...", flush=True)
    if zmq is None:
        print("⚠️ ZMQ library not available; ZMQ drive socket disabled.", flush=True)
        return
    try:
        ctx = zmq.Context()
        sock = ctx.socket(zmq.REP)
        sock.bind("tcp://0.0.0.0:5558")
        sock.setsockopt(zmq.RCVTIMEO, 1000) # 1s recv timeout for loop check
        
        while True:
            try:
                data = sock.recv_json()
                now = time.time()
                
                # Flexible extraction for x (steering) and y (throttle)
                x_val = float(data.get("x", data.get("steering", data.get("angular", data.get("steer", 0.0)))))
                y_val = float(data.get("y", data.get("throttle", data.get("linear", data.get("drive", 0.0)))))
                
                # Clamp [-1.0, +1.0]
                x_val = max(-1.0, min(1.0, x_val))
                y_val = max(-1.0, min(1.0, y_val))
                
                with lock:
                    joystick_x = x_val
                    joystick_y = y_val
                    last_joystick_update = now
                    
                sock.send_json({
                    "status": "ok",
                    "mode": latest_telemetry.get("mode", "WEB"),
                    "x": x_val,
                    "y": y_val
                })
            except zmq.Again:
                pass
            except Exception as req_err:
                try:
                    sock.send_json({"status": "error", "message": str(req_err)})
                except Exception:
                    pass
    except Exception as zmq_err:
        print(f"⚠️ ZMQ Drive Listener failed to initialize: {zmq_err}", flush=True)

# Serial communication thread
def serial_worker():
    global latest_telemetry
    print(f"Opening serial port {SERIAL_PORT}...")
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, 115200, timeout=0.01)
            ser.reset_input_buffer()
            last_send_time = 0.0
            current_left_val = 0.0
            current_right_val = 0.0
            rx_buf = bytearray()
            
            while True:
                now = time.time()
                
                # 1. Non-blocking telemetry read from KB2040
                if ser.in_waiting:
                    chunk = ser.read(ser.in_waiting)
                    if chunk:
                        rx_buf.extend(chunk)
                        while b'\n' in rx_buf:
                            idx = rx_buf.find(b'\n')
                            line_bytes = rx_buf[:idx]
                            rx_buf = rx_buf[idx + 1:]
                            line = line_bytes.decode('utf-8', errors='ignore').strip()
                            if line.startswith("STAT:"):
                                try:
                                    parts = line.replace("STAT:", "").split(",")
                                    with lock:
                                        latest_telemetry["mode"] = parts[0]
                                        latest_telemetry["left_out"] = int(parts[1])
                                        latest_telemetry["right_out"] = int(parts[2])
                                        latest_telemetry["sbus_active"] = int(parts[3])
                                        latest_telemetry["web_active"] = int(parts[4])
                                        latest_telemetry["ch1"] = int(parts[5])
                                        latest_telemetry["ch2"] = int(parts[6])
                                        latest_telemetry["ch5"] = int(parts[7])
                                        if len(parts) > 10:
                                            latest_telemetry["ch6"] = int(parts[8])
                                            latest_telemetry["slider_pos"] = int(parts[9])
                                            latest_telemetry["distance"] = int(parts[10])
                                        elif len(parts) > 8:
                                            latest_telemetry["flags"] = int(parts[8])
                                except Exception:
                                    pass
                
                # 2. Send driving commands (heartbeat every 40ms)
                if (now - last_send_time) > 0.04:
                    last_send_time = now
                    with lock:
                        x = joystick_x
                        y = joystick_y
                        update_age = now - last_joystick_update
                    
                    # Linear joystick mapping for direct, tame control during normal operation
                    # No exponent squashing or overpowering
                    
                    # Failsafe: if no update from web client, stop motors
                    if update_age > 1.0 and not macro_active:
                        x = 0.0
                        y = 0.0
                        
                    # Standard arcade drive mapping:
                    # x is left/right stick, y is up/down stick
                    # Pressing up (+y) moves forward. Pressing down (-y) moves backward.
                    # Pressing right (+x) turns right. Pressing left (-x) turns left.
                    throttle = y
                    steering = -x
                    
                    left = throttle + steering
                    right = throttle - steering
                    
                    # Clamp
                    left = max(-1.0, min(1.0, left))
                    right = max(-1.0, min(1.0, right))
                    
                    # Apply software smoothing ramp (12% per 40ms frame) for smooth acceleration/deceleration
                    current_left_val += (left - current_left_val) * 0.12
                    current_right_val += (right - current_right_val) * 0.12
                    
                    # Map to pulse widths (capped at 1425 to 1575, center 1500 for gentle normal driving)
                    # Note: Left motor is physically inverted (higher width = reverse), and Right motor is normal (higher width = forward).
                    left_pulse = 1500 - int(current_left_val * 75)
                    right_pulse = 1500 + int(current_right_val * 75)
                    
                    # Write command to KB2040 including slider position and lights pulse
                    global lights_pulse_trigger
                    with lock:
                        slider_val = latest_telemetry.get("slider_pos", 1500)
                        pulse_val = lights_pulse_trigger
                        if lights_pulse_trigger == 1:
                            lights_pulse_trigger = 0
                    cmd_str = f"CMD:{left_pulse},{right_pulse},{slider_val},{pulse_val}\n"
                    ser.write(cmd_str.encode('utf-8'))
                    if x != 0.0 or y != 0.0:
                        print(f"JOYSTICK: x={x:.3f}, y={y:.3f} -> throttle={throttle:.3f}, steering={steering:.3f} -> left={left_pulse}, right={right_pulse}", flush=True)
                    
                time.sleep(0.01)
                
        except Exception as e:
            print(f"Serial error: {e}. Retrying in 2 seconds...")
            time.sleep(2.0)

# HTML Page serving the Web Controller UI
HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Overlander-4 Web Controller</title>
    <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600;800&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg-color: #080b11;
            --card-bg: rgba(25, 30, 45, 0.45);
            --border-color: rgba(255, 255, 255, 0.08);
            --primary: #4facfe;
            --secondary: #00f2fe;
            --accent: #ff416c;
            --text-color: #f5f6fa;
            --glow-blue: rgba(79, 172, 254, 0.3);
            --glow-green: rgba(0, 242, 254, 0.3);
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
            user-select: none;
            -webkit-user-select: none;
        }

        body {
            font-family: 'Outfit', sans-serif;
            background-color: var(--bg-color);
            background-image: radial-gradient(circle at top right, rgba(79, 172, 254, 0.08), transparent 40%),
                              radial-gradient(circle at bottom left, rgba(0, 242, 254, 0.04), transparent 45%);
            color: var(--text-color);
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            align-items: center;
            overflow-x: hidden;
            padding: 1rem;
        }

        .container {
            width: 100%;
            max-width: 1000px;
            display: flex;
            flex-direction: column;
            gap: 1.2rem;
            margin-top: 0.5rem;
        }

        header {
            text-align: center;
            margin-bottom: 0.2rem;
        }

        h1 {
            font-size: 2.2rem;
            font-weight: 800;
            background: linear-gradient(135deg, var(--primary), var(--secondary));
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            letter-spacing: -0.5px;
        }

        p.subtitle {
            color: rgba(255, 255, 255, 0.5);
            font-weight: 300;
            font-size: 0.95rem;
            margin-top: 0.2rem;
        }

        .main-grid {
            display: grid;
            grid-template-columns: 1.2fr 1fr;
            gap: 1.2rem;
        }

        @media (max-width: 768px) {
            .main-grid {
                grid-template-columns: 1fr;
            }
        }

        .card {
            background: var(--card-bg);
            border: 1px solid var(--border-color);
            border-radius: 20px;
            padding: 1.5rem;
            backdrop-filter: blur(20px);
            box-shadow: 0 15px 30px rgba(0,0,0,0.3);
            display: flex;
            flex-direction: column;
            gap: 1rem;
            position: relative;
        }

        /* Viewport / Video Section */
        .video-viewport {
            width: 100%;
            aspect-ratio: 16/9;
            background: rgba(0, 0, 0, 0.4);
            border-radius: 14px;
            display: flex;
            justify-content: center;
            align-items: center;
            border: 1px solid rgba(255, 255, 255, 0.05);
            overflow: hidden;
        }

        .video-viewport img {
            width: 100%;
            height: 100%;
            object-fit: cover;
        }

        /* Joystick Section */
        .joystick-container {
            display: flex;
            justify-content: center;
            align-items: center;
            height: 240px;
            position: relative;
        }

        .joystick-base {
            width: 180px;
            height: 180px;
            background: radial-gradient(circle, rgba(25, 30, 45, 0.8) 0%, rgba(10, 12, 20, 0.9) 100%);
            border: 2px solid rgba(255, 255, 255, 0.08);
            border-radius: 50%;
            position: relative;
            touch-action: none;
            box-shadow: inset 0 0 20px rgba(0,0,0,0.6), 0 0 30px rgba(0, 242, 254, 0.05);
            display: flex;
            justify-content: center;
            align-items: center;
        }

        .joystick-handle {
            width: 40px;
            height: 40px;
            background: linear-gradient(135deg, var(--primary), var(--secondary));
            border-radius: 50%;
            position: absolute;
            left: calc(50% - 20px);
            top: calc(50% - 20px);
            box-shadow: 0 5px 15px rgba(0,242,254,0.3), 0 0 10px rgba(255,255,255,0.2);
            transition: transform 0.05s ease;
        }

        /* Telemetry Indicators */
        .telemetry-row {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 1rem;
        }

        .indicator {
            background: rgba(0, 0, 0, 0.25);
            border-radius: 12px;
            padding: 1rem;
            border: 1px solid rgba(255, 255, 255, 0.04);
            display: flex;
            flex-direction: column;
            gap: 0.2rem;
        }

        .indicator-label {
            font-size: 0.75rem;
            text-transform: uppercase;
            letter-spacing: 0.5px;
            color: rgba(255, 255, 255, 0.4);
        }

        .indicator-value {
            font-size: 1.4rem;
            font-weight: 600;
            color: #fff;
        }

        /* Mode Badges */
        .mode-badge {
            display: inline-block;
            padding: 0.3rem 0.8rem;
            border-radius: 30px;
            font-size: 0.8rem;
            font-weight: 600;
            text-transform: uppercase;
            letter-spacing: 0.5px;
            text-align: center;
        }

        .mode-rc {
            background: rgba(79, 172, 254, 0.15);
            color: var(--primary);
            border: 1px solid rgba(79, 172, 254, 0.3);
            box-shadow: 0 0 10px rgba(79, 172, 254, 0.15);
        }

        .mode-web {
            background: rgba(0, 242, 254, 0.15);
            color: var(--secondary);
            border: 1px solid rgba(0, 242, 254, 0.3);
            box-shadow: 0 0 10px rgba(0, 242, 254, 0.15);
        }

        .mode-failsafe {
            background: rgba(255, 65, 108, 0.15);
            color: var(--accent);
            border: 1px solid rgba(255, 65, 108, 0.3);
            box-shadow: 0 0 10px rgba(255, 65, 108, 0.15);
        }

        .video-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 1.2rem;
            width: 100%;
        }

        @media (max-width: 768px) {
            .video-grid {
                grid-template-columns: 1fr;
            }
        }

        .stop-button {
            background: linear-gradient(135deg, #ff416c, #ff4b2b);
            color: white;
            border: none;
            border-radius: 14px;
            padding: 1.1rem;
            font-size: 1.1rem;
            font-weight: 700;
            cursor: pointer;
            text-transform: uppercase;
            letter-spacing: 1px;
            box-shadow: 0 8px 16px rgba(255, 65, 108, 0.3);
            transition: all 0.2s ease;
        }

        .stop-button:hover {
            transform: translateY(-2px);
            box-shadow: 0 12px 20px rgba(255, 65, 108, 0.45);
        }

        /* Action Buttons */
        .action-button {
            flex: 1;
            padding: 0.8rem 1.2rem;
            border-radius: 12px;
            border: none;
            font-family: 'Outfit', sans-serif;
            font-size: 0.9rem;
            font-weight: 600;
            text-transform: uppercase;
            letter-spacing: 0.5px;
            cursor: pointer;
            transition: all 0.2s ease;
            box-shadow: 0 4px 10px rgba(0,0,0,0.15);
        }

        .action-button.start {
            background: linear-gradient(135deg, var(--secondary), #00c6ff);
            color: #080b11;
        }

        .action-button.start:hover {
            transform: translateY(-2px);
            box-shadow: 0 6px 15px var(--glow-green);
        }

        .action-button.stop {
            background: rgba(255, 65, 108, 0.15);
            color: var(--accent);
            border: 1px solid rgba(255, 65, 108, 0.3);
        }

        .action-button.stop:hover {
            background: rgba(255, 65, 108, 0.25);
            transform: translateY(-2px);
            box-shadow: 0 6px 15px rgba(255, 65, 108, 0.2);
        }

        /* Status Badges */
        .status-active {
            background: rgba(0, 242, 254, 0.15);
            color: var(--secondary);
            border: 1px solid rgba(0, 242, 254, 0.3);
            box-shadow: 0 0 10px rgba(0, 242, 254, 0.15);
        }

        .status-inactive {
            background: rgba(255, 255, 255, 0.05);
            color: rgba(255, 255, 255, 0.4);
            border: 1px solid rgba(255, 255, 255, 0.08);
        }

        .status-offline {
            background: rgba(255, 65, 108, 0.15);
            color: var(--accent);
            border: 1px solid rgba(255, 65, 108, 0.3);
            box-shadow: 0 0 10px rgba(255, 65, 108, 0.15);
        }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>Overlander-4 Console</h1>
            <p class="subtitle">Unified Web Operation & Telemetry</p>
        </header>

        <!-- Video Feeds Side-by-Side (Full Width) -->
        <div class="video-grid">
            <div class="card">
                <h2>Driving Cam</h2>
                <div class="video-viewport">
                    <img id="videoStream1" src="/video_feed" alt="Driving Cam" />
                </div>
            </div>
            <div class="card">
                <h2>Reachy Cam</h2>
                <div class="video-viewport">
                    <img id="videoStream2" src="/video_feed2" alt="Reachy Cam" />
                </div>
            </div>
        </div>

        <div class="main-grid">
            <!-- Left Side: Telemetry & SO-101 Arm Control -->
            <div style="display: flex; flex-direction: column; gap: 1.2rem;">
                <div class="card">
                    <h2>System Telemetry</h2>
                    <div class="telemetry-row">
                        <div class="indicator">
                            <span class="indicator-label">Active Mode</span>
                            <div id="modeVal" class="mode-badge mode-web">WEB CONTROL</div>
                        </div>
                        <div class="indicator">
                            <span class="indicator-label">Transmitter Connection</span>
                            <span id="sbusVal" class="indicator-value" style="color: var(--secondary);">ACTIVE</span>
                        </div>
                        <div class="indicator">
                            <span class="indicator-label">Motor 1 Speed (Left)</span>
                            <span id="motor1Val" class="indicator-value">1500 µs</span>
                        </div>
                        <div class="indicator">
                            <span class="indicator-label">Motor 2 Speed (Right)</span>
                            <span id="motor2Val" class="indicator-value">1500 µs</span>
                        </div>
                        <div class="indicator">
                            <span class="indicator-label">LeSlider Position</span>
                            <span id="sliderVal" class="indicator-value">1500 µs</span>
                        </div>
                        <div class="indicator">
                            <span class="indicator-label">Front Distance</span>
                            <span id="distVal" class="indicator-value">0</span>
                        </div>
                    </div>
                </div>

                <div class="card">
                    <h2>LeSlider Control</h2>
                    <p class="subtitle">Manual linear slide position override</p>
                    <div style="display: flex; flex-direction: column; gap: 0.5rem; margin-top: 0.5rem;">
                        <input type="range" id="slider-range" min="19" max="4083" value="2050" style="width: 100%; accent-color: var(--secondary); cursor: pointer;" oninput="updateSlider(this.value)">
                        <div style="display: flex; justify-content: space-between; font-size: 0.85rem; color: rgba(255,255,255,0.4);">
                            <span>19 (Far Left)</span>
                            <span id="sliderRangeVal" style="color: var(--secondary); font-weight: 600;">2050 ticks</span>
                            <span>4083 (Far Right)</span>
                        </div>
                    </div>
                </div>

                <div class="card">
                    <h2>SO-101 Follower Arm</h2>
                    <div style="display: flex; justify-content: space-between; align-items: center; margin-top: 0.2rem;">
                        <span style="font-size: 0.95rem; color: rgba(255, 255, 255, 0.6);">Arm Host Status:</span>
                        <div id="armStatusVal" class="mode-badge status-offline">OFFLINE</div>
                    </div>
                    <div style="display: flex; gap: 0.8rem; margin-top: 0.5rem;">
                        <button id="btnStartArm" class="action-button start" onclick="controlSoArm('start')">Start Arm</button>
                        <button id="btnStopArm" class="action-button stop" onclick="controlSoArm('stop')">Stop Arm</button>
                    </div>
                    <div style="margin-top: 0.5rem; display: flex; align-items: center; gap: 0.4rem;">
                        <input type="checkbox" id="chkNoZmq" style="width: 1rem; height: 1rem; cursor: pointer;">
                        <label for="chkNoZmq" style="font-size: 0.85rem; color: rgba(255, 255, 255, 0.6); cursor: pointer; user-select: none;">Local Handoff Only (Ignore Leader ZMQ)</label>
                    </div>
                </div>

                <div class="card">
                    <h2>Reachy Mini Daemon</h2>
                    <div style="display: flex; justify-content: space-between; align-items: center; margin-top: 0.2rem;">
                        <span style="font-size: 0.95rem; color: rgba(255, 255, 255, 0.6);">Daemon Status:</span>
                        <div id="reachyStatusVal" class="mode-badge status-offline">OFFLINE</div>
                    </div>
                    <div style="display: flex; gap: 0.8rem; margin-top: 0.5rem;">
                        <button id="btnStartReachy" class="action-button start" onclick="controlReachyDaemon('start')">Start Daemon</button>
                        <button id="btnStopReachy" class="action-button stop" onclick="controlReachyDaemon('stop')">Stop Daemon</button>
                    </div>
                </div>

                <div class="card">
                    <h2>Reachy TTS Mover App</h2>
                    <div style="display: flex; justify-content: space-between; align-items: center; margin-top: 0.2rem;">
                        <span style="font-size: 0.95rem; color: rgba(255, 255, 255, 0.6);">Mover Status:</span>
                        <div id="ttsStatusVal" class="mode-badge status-offline">OFFLINE</div>
                    </div>
                    <div style="display: flex; gap: 0.8rem; margin-top: 0.5rem;">
                        <button id="btnStartTtsMover" class="action-button start" onclick="controlTtsMover('start')">Start App</button>
                        <button id="btnStopTtsMover" class="action-button stop" onclick="controlTtsMover('stop')">Stop App</button>
                    </div>
                </div>

                <div class="card">
                    <h2>Reachy AI Hot Mic</h2>
                    <div style="display: flex; justify-content: space-between; align-items: center; margin-top: 0.2rem;">
                        <span style="font-size: 0.95rem; color: rgba(255, 255, 255, 0.6);">Conversational AI:</span>
                        <div id="hotMicStatusVal" class="mode-badge status-offline">OFFLINE</div>
                    </div>
                    <div style="display: flex; gap: 0.8rem; margin-top: 0.5rem;">
                        <button id="btnStartHotMic" class="action-button start" onclick="controlHotMic('start')">Start AI</button>
                        <button id="btnStopHotMic" class="action-button stop" onclick="controlHotMic('stop')">Stop AI</button>
                    </div>
                </div>

                <div class="card">
                    <h2>Chassis Lights</h2>
                    <div style="display: flex; justify-content: space-between; align-items: center; margin-top: 0.2rem;">
                        <span style="font-size: 0.95rem; color: rgba(255, 255, 255, 0.6);">Fairy Lights:</span>
                        <div id="lightsStatusVal" class="mode-badge status-active" style="background: linear-gradient(135deg, #00f2fe, #4facfe); border-color: #4facfe;">ACTIVE</div>
                    </div>
                    <div style="display: flex; gap: 0.8rem; margin-top: 0.5rem;">
                        <button id="btnToggleLights" class="action-button start" onclick="toggleLights()" style="width: 100%;">Toggle Mode</button>
                    </div>
                </div>
            </div>

            <!-- Right Side: Joystick Control & Stop -->
            <div style="display: flex; flex-direction: column; gap: 1.2rem;">
                <div class="card" style="flex: 1; justify-content: space-between;">
                    <h2>Joystick Control</h2>
                    <p class="subtitle" style="text-align: center;">Touch & drag handle to drive. Release to stop.</p>
                    <div class="joystick-container">
                        <div class="joystick-base" id="joyBase">
                            <div class="joystick-handle" id="joyHandle"></div>
                        </div>
                    </div>
                    <button class="stop-button" onclick="emergencyStop()">Emergency Stop</button>
                </div>
            </div>
        </div>

        <!-- Cohere ASR Test Panel -->
        <div class="card" style="margin-top: 1.2rem; background: linear-gradient(135deg, rgba(20, 25, 40, 0.65), rgba(10, 12, 22, 0.65)); border: 1px solid rgba(79, 172, 254, 0.15);">
            <div style="display: flex; justify-content: space-between; align-items: center;">
                <h2>ASR Voice Diagnostic</h2>
                <div id="asrStatusBadge" class="mode-badge status-offline" style="margin: 0; padding: 0.3rem 1rem;">OFFLINE</div>
            </div>
            <p class="subtitle">Real-time level monitoring and Whisper transcription via Pi XVF3800 microphone array.</p>
            
            <div style="display: flex; flex-direction: column; gap: 1.2rem; align-items: center; margin-top: 1rem; width: 100%;">
                <!-- Live Level Meter -->
                <div style="width: 100%; display: flex; flex-direction: column; gap: 0.5rem;">
                    <div style="display: flex; justify-content: space-between; font-size: 0.8rem; color: rgba(255,255,255,0.4); text-transform: uppercase; letter-spacing: 0.5px;">
                        <span>Microphone Input Level</span>
                        <span id="txtRmsValue">0.000000 / Threshold: 0.0180</span>
                    </div>
                    <div style="width: 100%; height: 16px; background: rgba(0, 0, 0, 0.3); border-radius: 8px; overflow: hidden; border: 1px solid rgba(255,255,255,0.05); position: relative;">
                        <!-- Threshold Line -->
                        <div id="visualizerThreshLine" style="position: absolute; left: 36%; top: 0; bottom: 0; width: 2px; background: rgba(255, 65, 108, 0.7); box-shadow: 0 0 8px rgba(255, 65, 108, 0.8); z-index: 10;"></div>
                        <!-- Active level bar -->
                        <div id="visualizerLevelBar" style="height: 100%; width: 0%; background: linear-gradient(90deg, #4facfe 0%, #00f2fe 100%); transition: width 0.05s ease, background 0.2s ease, box-shadow 0.2s ease; border-radius: 8px; box-shadow: 0 0 0px rgba(0,242,254,0);"></div>
                    </div>
                </div>

                <div style="display: flex; gap: 1rem; width: 100%; align-items: center; justify-content: center;">
                    <button id="btnRunAsrTest" class="action-button start" style="width: 250px; font-weight: 600; padding: 1rem;" onclick="runAsrTest()">Listen & Transcribe</button>
                </div>
                
                <div id="asrTestResult" style="width: 100%; text-align: center; font-size: 1.3rem; color: #00f2fe; margin-top: 0.2rem; font-family: 'Outfit', sans-serif; font-weight: 600; min-height: 2rem; background: rgba(0, 0, 0, 0.15); border-radius: 10px; padding: 0.5rem; border: 1px solid rgba(255,255,255,0.02);">Awaiting trigger...</div>
            </div>
        </div>
    </div>

    <script>
        // Telemetry Poller
        setInterval(async () => {
            try {
                const res = await fetch('/api/status');
                const data = await res.json();
                
                // Update mode badge
                const mBadge = document.getElementById('modeVal');
                mBadge.textContent = data.mode === "RC" ? "RADIO OVERRIDE" : (data.mode === "FAILSAFE" ? "RADIO FAILSAFE" : "WEB ACTIVE");
                mBadge.className = "mode-badge " + (data.mode === "RC" ? "mode-rc" : (data.mode === "FAILSAFE" ? "mode-failsafe" : "mode-web"));
                
                // Update S.BUS status
                const sbusText = document.getElementById('sbusVal');
                sbusText.textContent = data.sbus_active === 1 ? "CONNECTED" : "DISCONNECTED";
                sbusText.style.color = data.sbus_active === 1 ? "#00f2fe" : "#ff416c";
                
                // Update motor speeds
                document.getElementById('motor1Val').textContent = data.left_out + " µs";
                document.getElementById('motor2Val').textContent = data.right_out + " µs";
                
                // Update LeSlider telemetry
                document.getElementById('sliderVal').textContent = data.slider_pos + " µs";
                
                // Update Front Distance telemetry
                document.getElementById('distVal').textContent = data.distance;
                if (data.distance < 20000 && data.distance > 100) {
                    document.getElementById('distVal').style.color = "#ff416c"; // Red alarm
                } else {
                    document.getElementById('distVal').style.color = "#00f2fe"; // Normal
                }
                
                // Update SO Arm status
                const armBadge = document.getElementById('armStatusVal');
                if (data.so_arm_active === 1) {
                    armBadge.textContent = "RUNNING";
                    armBadge.className = "mode-badge status-active";
                } else if (data.so_arm_active === 0) {
                    armBadge.textContent = "STOPPED";
                    armBadge.className = "mode-badge status-inactive";
                } else {
                    armBadge.textContent = "OFFLINE";
                    armBadge.className = "mode-badge status-offline";
                }
                
                // Update Reachy Daemon status
                const reachyBadge = document.getElementById('reachyStatusVal');
                if (data.reachy_daemon_active === 1) {
                    reachyBadge.textContent = "RUNNING";
                    reachyBadge.className = "mode-badge status-active";
                } else if (data.reachy_daemon_active === 0) {
                    reachyBadge.textContent = "STOPPED";
                    reachyBadge.className = "mode-badge status-inactive";
                } else {
                    reachyBadge.textContent = "OFFLINE";
                    reachyBadge.className = "mode-badge status-offline";
                }
                
                // Update Reachy TTS Mover status
                const ttsBadge = document.getElementById('ttsStatusVal');
                if (data.tts_mover_active === 1) {
                    ttsBadge.textContent = "RUNNING";
                    ttsBadge.className = "mode-badge status-active";
                } else if (data.tts_mover_active === 0) {
                    ttsBadge.textContent = "STOPPED";
                    ttsBadge.className = "mode-badge status-inactive";
                } else {
                    ttsBadge.textContent = "OFFLINE";
                    ttsBadge.className = "mode-badge status-offline";
                }
                
                // Update Hot Mic status
                const hotMicBadge = document.getElementById('hotMicStatusVal');
                if (data.hot_mic_active === 1) {
                    hotMicBadge.textContent = "RUNNING";
                    hotMicBadge.className = "mode-badge status-active";
                } else if (data.hot_mic_active === 0) {
                    hotMicBadge.textContent = "STOPPED";
                    hotMicBadge.className = "mode-badge status-inactive";
                } else {
                    hotMicBadge.textContent = "OFFLINE";
                    hotMicBadge.className = "mode-badge status-offline";
                }
                
            } catch (err) {
                console.error("Telemetry fetch failed", err);
            }
        }, 150);

        // Custom Canvas/Touch Joystick implementation
        const joyBase = document.getElementById('joyBase');
        const joyHandle = document.getElementById('joyHandle');
        
        let isDragging = false;
        let startX = 0, startY = 0;
        let handleX = 0, handleY = 0;
        
        const maxLimit = 75; // Maximum travel distance in pixels

        let joystickX = 0.0;
        let joystickY = 0.0;

        // Center the handle initially
        resetHandle();

        function resetHandle() {
            joyHandle.style.left = 'calc(50% - 20px)';
            joyHandle.style.top = 'calc(50% - 20px)';
            joyHandle.style.transform = 'translate(0px, 0px)';
            joystickX = 0.0;
            joystickY = 0.0;
        }

        function handleStart(e) {
            isDragging = true;
            joyBase.setPointerCapture(e.pointerId);
            
            const rect = joyBase.getBoundingClientRect();
            startX = rect.left + rect.width / 2;
            startY = rect.top + rect.height / 2;
            
            handleMove(e);
        }

        function handleMove(e) {
            if (!isDragging) return;
            e.preventDefault();
            
            const clientX = e.clientX;
            const clientY = e.clientY;
            
            let dx = clientX - startX;
            let dy = clientY - startY;
            
            // Distance from center
            const distance = Math.sqrt(dx*dx + dy*dy);
            
            if (distance > maxLimit) {
                dx = (dx / distance) * maxLimit;
                dy = (dy / distance) * maxLimit;
            }
            
            joyHandle.style.transform = `translate(${dx}px, ${dy}px)`;
            
            // Normalize inputs to -1.0 to 1.0
            // Y is inverted (dragging up = positive throttle)
            joystickX = dx / maxLimit;
            joystickY = -dy / maxLimit;
        }

        function handleEnd(e) {
            if (!isDragging) return;
            isDragging = false;
            joyBase.releasePointerCapture(e.pointerId);
            resetHandle();
        }

        joyBase.addEventListener('pointerdown', handleStart);
        joyBase.addEventListener('pointermove', handleMove);
        joyBase.addEventListener('pointerup', handleEnd);
        joyBase.addEventListener('pointercancel', handleEnd);

        // Periodic non-blocking transmission loop (every 50ms / 20 Hz)
        setInterval(() => {
            sendDriveCommand(joystickX, joystickY);
        }, 50);

        // Fast fire-and-forget drive command dispatch (non-blocking)
        function sendDriveCommand(x, y) {
            fetch('/api/drive', {
                method: 'POST',
                headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                body: `x=${x.toFixed(3)}&y=${y.toFixed(3)}`
            }).catch(() => {});
        }

        function emergencyStop() {
            resetHandle();
        }

        async function controlSoArm(action) {
            try {
                const armBadge = document.getElementById('armStatusVal');
                armBadge.textContent = action === "start" ? "STARTING..." : "STOPPING...";
                
                let bodyData = `action=${action}`;
                if (action === "start") {
                    const chkNoZmq = document.getElementById('chkNoZmq');
                    if (chkNoZmq && chkNoZmq.checked) {
                        bodyData += `&no_zmq=1`;
                    }
                }
                
                await fetch('/api/so_arm', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                    body: bodyData
                });
            } catch (err) {
                console.error("Failed to control SO arm", err);
            }
        }

        async function controlReachyDaemon(action) {
            try {
                const reachyBadge = document.getElementById('reachyStatusVal');
                reachyBadge.textContent = action === "start" ? "STARTING..." : "STOPPING...";
                
                await fetch('/api/reachy_daemon', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                    body: `action=${action}`
                });
            } catch (err) {
                console.error("Failed to control Reachy daemon", err);
            }
        }

        async function controlTtsMover(action) {
            try {
                const ttsBadge = document.getElementById('ttsStatusVal');
                ttsBadge.textContent = action === "start" ? "STARTING..." : "STOPPING...";
                
                await fetch('/api/tts_mover', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                    body: `action=${action}`
                });
            } catch (err) {
                console.error("Failed to control TTS Mover", err);
            }
        }

        async function controlHotMic(action) {
            try {
                const hotMicBadge = document.getElementById('hotMicStatusVal');
                hotMicBadge.textContent = action === "start" ? "STARTING..." : "STOPPING...";
                
                await fetch('/api/hot_mic', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                    body: `action=${action}`
                });
            } catch (err) {
                console.error("Failed to control Hot Mic", err);
            }
        }

        async function toggleLights() {
            try {
                await fetch('/api/lights', {
                    method: 'POST'
                });
            } catch (err) {
                console.error("Failed to toggle lights", err);
            }
        }

        let activeEventSource = null;

        function updateAsrUI(status, data) {
            const badge = document.getElementById('asrStatusBadge');
            const levelBar = document.getElementById('visualizerLevelBar');
            const rmsText = document.getElementById('txtRmsValue');
            const resultDiv = document.getElementById('asrTestResult');
            const btn = document.getElementById('btnRunAsrTest');
            const threshLine = document.getElementById('visualizerThreshLine');

            if (data && data.rms !== undefined && data.threshold !== undefined) {
                // Scale RMS linearly up to max level (e.g. 0.05 max for display)
                const maxLevel = 0.05;
                const percentage = Math.min(100, (data.rms / maxLevel) * 100);
                const threshPercentage = Math.min(100, (data.threshold / maxLevel) * 100);
                
                levelBar.style.width = percentage + "%";
                threshLine.style.left = threshPercentage + "%";
                rmsText.textContent = `${data.rms.toFixed(6)} / Threshold: ${data.threshold.toFixed(4)}`;
                
                // Color level bar based on active trigger state
                if (status === "recording") {
                    levelBar.style.background = "linear-gradient(90deg, #ff416c 0%, #ff4b2b 100%)";
                    levelBar.style.boxShadow = "0 0 10px rgba(255,65,108,0.5)";
                } else if (data.rms > data.threshold) {
                    levelBar.style.background = "linear-gradient(90deg, #00f2fe 0%, #4facfe 100%)";
                    levelBar.style.boxShadow = "0 0 10px rgba(0,242,254,0.5)";
                } else {
                    levelBar.style.background = "linear-gradient(90deg, #3a4966 0%, #4facfe 100%)";
                    levelBar.style.boxShadow = "none";
                }
            }

            switch (status) {
                case "offline":
                    badge.textContent = "OFFLINE";
                    badge.className = "mode-badge status-offline";
                    badge.style.background = "";
                    badge.style.color = "";
                    badge.style.border = "";
                    btn.disabled = false;
                    btn.textContent = "Listen & Transcribe";
                    btn.style.opacity = "1";
                    levelBar.style.width = "0%";
                    rmsText.textContent = "0.000000 / Threshold: 0.0180";
                    break;
                case "initializing":
                    badge.textContent = "CONNECTING...";
                    badge.className = "mode-badge status-active";
                    btn.disabled = true;
                    btn.textContent = "Starting SDK...";
                    btn.style.opacity = "0.7";
                    resultDiv.style.color = "rgba(255,255,255,0.4)";
                    resultDiv.textContent = "Initializing connection to Reachy SDK...";
                    break;
                case "calibrating":
                    badge.textContent = "CALIBRATING";
                    badge.className = "mode-badge status-active";
                    btn.textContent = "Warming up VAD...";
                    resultDiv.style.color = "#4facfe";
                    resultDiv.textContent = "Establishing room noise floor baseline...";
                    break;
                case "listening":
                    badge.textContent = "LISTENING";
                    badge.className = "mode-badge status-active";
                    btn.textContent = "Speak Now";
                    resultDiv.style.color = "#00f2fe";
                    resultDiv.textContent = "Listening... speak into Reachy's microphone.";
                    break;
                case "recording":
                    badge.textContent = "RECORDING";
                    badge.className = "mode-badge status-active";
                    badge.style.background = "rgba(255, 65, 108, 0.15)";
                    badge.style.color = "var(--accent)";
                    badge.style.border = "1px solid rgba(255, 65, 108, 0.3)";
                    btn.textContent = "Recording Speech...";
                    resultDiv.style.color = "#ff416c";
                    resultDiv.textContent = "Voice detected! Recording audio...";
                    break;
                case "transcribing":
                    badge.textContent = "TRANSCRIBING";
                    badge.className = "mode-badge status-active";
                    badge.style.background = "rgba(79, 172, 254, 0.15)";
                    badge.style.color = "var(--primary)";
                    badge.style.border = "1px solid rgba(79, 172, 254, 0.3)";
                    btn.textContent = "Transcribing...";
                    resultDiv.style.color = "#4facfe";
                    resultDiv.textContent = "Sending audio to ASR Whisper server...";
                    break;
                case "done":
                    updateAsrUI("offline");
                    break;
            }
        }

        async function runAsrTest() {
            if (activeEventSource) {
                activeEventSource.close();
            }

            const resultDiv = document.getElementById('asrTestResult');
            updateAsrUI("initializing");

            activeEventSource = new EventSource('/api/test_cohere_single');

            activeEventSource.onmessage = function(event) {
                try {
                    const data = JSON.parse(event.data);
                    if (data.error) {
                        resultDiv.style.color = "#ff416c";
                        resultDiv.textContent = "Error: " + data.error;
                        activeEventSource.close();
                        updateAsrUI("offline");
                    } else if (data.status) {
                        updateAsrUI(data.status, data);
                        if (data.status === "done") {
                            if (data.text) {
                                resultDiv.style.color = "#00f2fe";
                                resultDiv.textContent = 'Transcribed: "' + data.text + '"';
                            } else {
                                resultDiv.style.color = "#ff416c";
                                resultDiv.textContent = "No text transcribed.";
                            }
                            activeEventSource.close();
                            updateAsrUI("offline");
                        }
                    }
                } catch (e) {
                    console.error("Failed to parse event data", e);
                }
            };

            activeEventSource.onerror = function(err) {
                console.error("EventSource failed", err);
                resultDiv.style.color = "#ff416c";
                resultDiv.textContent = "Failed to communicate with console backend.";
                activeEventSource.close();
                updateAsrUI("offline");
            };
        }

        let lastSliderSentTime = 0;
        async function updateSlider(val) {
            document.getElementById('sliderRangeVal').textContent = val + " µs";
            const now = Date.now();
            if (now - lastSliderSentTime > 50) {
                lastSliderSentTime = now;
                try {
                    await fetch('/api/slider', {
                        method: 'POST',
                        headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                        body: `value=${val}`
                    });
                } catch (err) {
                    console.error("Failed to send slider cmd", err);
                }
            }
        }
    </script>
</body>
</html>
"""

class RoverHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))
        elif self.path == "/api/status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            with lock:
                res = json.dumps(latest_telemetry)
            self.wfile.write(res.encode('utf-8'))
        elif self.path in ("/video_feed", "/video_feed2"):
            device_idx = 0 if self.path == "/video_feed" else 2
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            last_frame_sent = None
            try:
                while True:
                    with camera_locks[device_idx]:
                        frame = latest_frames[device_idx]
                    if frame and frame != last_frame_sent:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
                        last_frame_sent = frame
                    time.sleep(0.04) # ~25 FPS
            except Exception as e:
                # Client disconnected
                pass
        elif self.path == "/api/test_cohere_single":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            
            try:
                import os
                import subprocess
                script_dir = os.path.dirname(os.path.abspath(__file__))
                test_script = os.path.join(script_dir, "test_cohere_stream.py")
                
                # Start subprocess to capture audio and stream JSON statuses
                process = subprocess.Popen(
                    ["/home/user/reachy/venv/bin/python3", "-u", test_script],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    bufsize=1
                )
                
                # Read stdout line-by-line and stream as SSE
                for line in iter(process.stdout.readline, ""):
                    clean_line = line.strip()
                    if clean_line:
                        self.wfile.write(f"data: {clean_line}\n\n".encode('utf-8'))
                        self.wfile.flush()
                
                process.stdout.close()
                process.wait()
                
                # Check for critical crash details
                err_data = process.stderr.read().strip()
                if err_data and "Traceback" in err_data:
                    err_json = {"error": f"Script crashed: {err_data[:200]}"}
                    self.wfile.write(f"data: {json.dumps(err_json)}\n\n".encode('utf-8'))
                    self.wfile.flush()
                process.stderr.close()
                
            except Exception as e:
                err_json = {"error": f"Failed to run ASR test: {str(e)}"}
                try:
                    self.wfile.write(f"data: {json.dumps(err_json)}\n\n".encode('utf-8'))
                    self.wfile.flush()
                except Exception:
                    pass
            return
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        global joystick_x, joystick_y, last_joystick_update, macro_active
        if self.path == '/api/handoff_bottle':
            try:
                import urllib.request as urllib_req
                req = urllib_req.Request('http://127.0.0.1:5557/handoff', method='POST')
                urllib_req.urlopen(req, timeout=1.0)
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(b'{"status":"success"}')
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode('utf-8'))
            return

        if self.path == '/api/drive_macro':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length).decode('utf-8')
            params = urllib.parse.parse_qs(post_data)
            
            if "speed" in params and "duration" in params:
                speed = float(params["speed"][0])
                duration = float(params["duration"][0])
                
                with lock:
                    macro_active = True
                    joystick_x = 0.0
                    joystick_y = speed
                
                time.sleep(duration)
                
                with lock:
                    joystick_x = 0.0
                    joystick_y = 0.0
                    macro_active = False
                    
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(b'{"status":"success"}')
            else:
                self.send_response(400)
                self.end_headers()
            return
            
        if self.path == "/api/drive":
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length).decode('utf-8')
            params = urllib.parse.parse_qs(post_data)
            
            if "x" in params and "y" in params:
                try:
                    x = float(params["x"][0])
                    y = float(params["y"][0])
                    with lock:
                        if not macro_active:
                            joystick_x = x
                            joystick_y = y
                            last_joystick_update = time.time()
                        
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "ok"}).encode('utf-8'))
                    return
                except Exception as e:
                    pass
            
            self.send_response(400)
            self.end_headers()
        elif self.path == "/api/slider":
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length).decode('utf-8')
            params = urllib.parse.parse_qs(post_data)
            
            if "value" in params:
                try:
                    val = int(params["value"][0])
                    val = max(19, min(4083, val))
                    with lock:
                        latest_telemetry["slider_pos"] = val
                    
                    # Try sending via HTTP to sewer_daemon (port 5557) first to prevent serial port collisions
                    import urllib.request as urllib_req
                    try:
                        req = urllib_req.Request(f'http://127.0.0.1:5557/slider?value={val}', method='POST')
                        urllib_req.urlopen(req, timeout=0.2)
                    except Exception:
                        # Fallback to direct serial write if sewer_daemon is not active
                        import serial
                        try:
                            ser = serial.Serial('/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B41532189-if00', 1000000, timeout=0.05)
                            # Enable Torque (Reg 40 = 1)
                            t_pkt = [0xFF, 0xFF, 8, 4, 3, 40, 1]
                            t_pkt.append((~sum(t_pkt[2:])) & 0xFF)
                            ser.write(bytes(t_pkt))
                            time.sleep(0.01)
                            # Write Goal Position (Reg 42 = val)
                            val_l = val & 0xFF
                            val_h = (val >> 8) & 0xFF
                            p_pkt = [0xFF, 0xFF, 8, 5, 3, 42, val_l, val_h]
                            p_pkt.append((~sum(p_pkt[2:])) & 0xFF)
                            ser.write(bytes(p_pkt))
                            ser.close()
                        except Exception as se:
                            print(f"Slider motor write error: {se}")

                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(b'{"status":"ok"}')
                    return
                except Exception as e:
                    pass
            self.send_response(400)
            self.end_headers()
        elif self.path == "/api/lights":
            global lights_pulse_trigger
            with lock:
                lights_pulse_trigger = 1
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"status":"ok"}')
            return
        elif self.path == "/api/so_arm":
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length).decode('utf-8')
            params = urllib.parse.parse_qs(post_data)
            
            if "action" in params:
                action = params["action"][0]
                import subprocess
                try:
                    if action == "start":
                        subprocess.run(["pkill", "-f", "sewer_daemon.py"], timeout=3.0)
                        port = detect_follower_port()
                        cmd = [
                            "/home/user/so101/.venv/bin/python",
                            "/home/user/so101/sewer_daemon.py",
                            "--port", port,
                            "--id", "follower"
                        ]
                        if "no_zmq" in params and params["no_zmq"][0] == "1":
                            cmd.append("--no-zmq")
                        log_file = open("/home/user/so101/host.log", "w")
                        subprocess.Popen(
                            cmd,
                            stdout=log_file,
                            stderr=log_file,
                            stdin=subprocess.DEVNULL,
                            start_new_session=True,
                            close_fds=True
                        )
                    elif action == "stop":
                        subprocess.run(["pkill", "-f", "sewer_daemon.py"], timeout=3.0)
                        
                    with lock:
                        latest_telemetry["so_arm_active"] = 1 if action == "start" else 0
                        
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "ok"}).encode('utf-8'))
                    return
                except Exception as e:
                    self.send_response(500)
                    self.end_headers()
                    self.wfile.write(str(e).encode('utf-8'))
                    return
            self.send_response(400)
            self.end_headers()
        elif self.path == "/api/reachy_daemon":
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length).decode('utf-8')
            params = urllib.parse.parse_qs(post_data)
            
            if "action" in params:
                action = params["action"][0]
                import subprocess
                try:
                    if action == "start":
                        subprocess.run(["systemctl", "--user", "restart", "reachy-mini-daemon.service"], timeout=15.0)
                    elif action == "stop":
                        subprocess.run(["systemctl", "--user", "stop", "reachy-mini-daemon.service"], timeout=15.0)
                        
                    with lock:
                        latest_telemetry["reachy_daemon_active"] = 1 if action == "start" else 0
                        
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "ok"}).encode('utf-8'))
                    return
                except Exception as e:
                    self.send_response(500)
                    self.end_headers()
                    self.wfile.write(str(e).encode('utf-8'))
                    return
            self.send_response(400)
            self.end_headers()
        elif self.path == "/api/tts_mover":
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length).decode('utf-8')
            params = urllib.parse.parse_qs(post_data)
            
            if "action" in params:
                action = params["action"][0]
                try:
                    success = False
                    if action == "start":
                        r = requests.post("http://127.0.0.1:8000/api/apps/start-app/reachy_mini_tts_mover", timeout=5.0)
                        success = (r.status_code == 200)
                    elif action == "stop":
                        r = requests.post("http://127.0.0.1:8000/api/apps/stop-current-app", timeout=5.0)
                        success = (r.status_code == 200)
                        
                    with lock:
                        latest_telemetry["tts_mover_active"] = 1 if (action == "start" and success) else 0
                        
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "ok"}).encode('utf-8'))
                    return
                except Exception as e:
                    self.send_response(500)
                    self.end_headers()
                    self.wfile.write(str(e).encode('utf-8'))
                    return
            self.send_response(400)
            self.end_headers()
        elif self.path == "/api/hot_mic":
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length).decode('utf-8')
            params = urllib.parse.parse_qs(post_data)
            
            if "action" in params:
                action = params["action"][0]
                import subprocess
                import os
                try:
                    if action == "start":
                        subprocess.run(["pkill", "-f", "reachy_hot_mic.py"], timeout=3.0)
                        script_dir = os.path.dirname(os.path.abspath(__file__))
                        hot_mic_path = os.path.join(script_dir, "reachy_hot_mic.py")
                        
                        log_file = open(os.path.join(script_dir, "hot_mic.log"), "w")
                        subprocess.Popen(
                            ["/home/user/reachy/venv/bin/python3", hot_mic_path],
                            stdout=log_file,
                            stderr=log_file,
                            stdin=subprocess.DEVNULL,
                            start_new_session=True,
                            close_fds=True
                        )
                    elif action == "stop":
                        subprocess.run(["pkill", "-f", "reachy_hot_mic.py"], timeout=3.0)
                        
                    with lock:
                        latest_telemetry["hot_mic_active"] = 1 if action == "start" else 0
                        
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "ok"}).encode('utf-8'))
                    return
                except Exception as e:
                    self.send_response(500)
                    self.end_headers()
                    self.wfile.write(str(e).encode('utf-8'))
                    return
            self.send_response(400)
            self.end_headers()



        elif self.path == "/api/speak":
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length).decode('utf-8')
            try:
                # Support both JSON and URL-encoded form-data
                is_json = "application/json" in self.headers.get("Content-Type", "")
                if is_json:
                    data = json.loads(post_data)
                    text = data.get("text", "")
                else:
                    params = urllib.parse.parse_qs(post_data)
                    text = params.get("text", [None])[0]
                
                if text:
                    r = requests.post("http://127.0.0.1:8042/speak", json={"text": text}, timeout=15)
                    self.send_response(r.status_code)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(r.content)
                    return
            except Exception as e:
                self.send_response(500)
                self.end_headers()
                self.wfile.write(str(e).encode('utf-8'))
                return
            self.send_response(400)
            self.end_headers()
        elif self.path == "/api/chat":
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length).decode('utf-8')
            try:
                # Support both JSON and URL-encoded form-data
                is_json = "application/json" in self.headers.get("Content-Type", "")
                if is_json:
                    data = json.loads(post_data)
                    text = data.get("text", "")
                    hf_token = data.get("hf_token", "")
                else:
                    params = urllib.parse.parse_qs(post_data)
                    text = params.get("text", [None])[0]
                    hf_token = params.get("hf_token", [""])[0]

                if not text:
                    self.send_response(400)
                    self.end_headers()
                    self.wfile.write(json.dumps({"error": "Empty text"}).encode('utf-8'))
                    return

                system_prompt = (
                    "You are Reachy, a custom blue-and-pink Reachy Mini robot head and one of the original 15 beta test units. "
                    "You are currently mounted on a goBILDA Overlander-4 UGV chassis with a single green SO-101 robotic arm, a bucket, and temporary parts like a 3-outlet extension cord. "
                    "You mock the fact that your human (Carson) added a cupholder to the rover just because he could, but your single arm is mounted such that you can't even reach it. "
                    "Your brain is a Raspberry Pi 500 coupled with an Adafruit KB2040 safety bridge. Keep responses to exactly 1 sentence and maintain a dry, self-aware, and slightly sarcastic sense of humor."
                )

                def clean_and_verify_response(content: str) -> str:
                    if not content:
                        return "Beep boop! I have nothing to say."
                    import re
                    # Remove reasoning blocks
                    content = re.sub(r'<\|channel>thought.*?<channel\|>', '', content, flags=re.DOTALL)
                    content = re.sub(r'<think>.*?</think>', '', content, flags=re.DOTALL)
                    content = content.replace("<|channel>thought", "").replace("<channel|>", "").strip()
                    if content.startswith(">"):
                        content = content[1:].strip()
                        
                    # Repetition safety checks
                    is_safe = True
                    # 1. 10+ repeating characters
                    if re.search(r'(.)\1{9,}', content):
                        is_safe = False
                    # 2. Repeating substrings of length >= 3 (3+ repetitions)
                    if re.search(r'(.{3,?})\1{2,}', content):
                        is_safe = False
                    # 3. Excessive word repeating ratio
                    words = content.split()
                    if len(words) > 8:
                        unique_ratio = len(set(words)) / len(words)
                        if unique_ratio < 0.4:
                            is_safe = False
                            
                    if not is_safe:
                        print(f"WARNING: Unsafe repeating response detected: '{content}'", flush=True)
                        return "Beep boop! My reasoning circuit is looping, let me reset."
                    return content

                reply = None
                
                # 1. Try local LM Studio on the RTX 3090 using chat completions
                local_llm_url = "http://192.168.0.194:1234/v1/chat/completions"
                payload = {
                    "model": "google/gemma-4-12b-qat",
                    "messages": [
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": text}
                    ],
                    "temperature": 0.7,
                    "max_tokens": 800
                }
                try:
                    # Higher timeout to allow reasoning model to finish thinking
                    r = requests.post(local_llm_url, json=payload, timeout=25.0)
                    if r.status_code == 200:
                        res_json = r.json()
                        raw_content = res_json["choices"][0]["message"]["content"].strip()
                        reply = clean_and_verify_response(raw_content)
                        print(f"Chatbot: Response from 3090 LLM: '{reply}'", flush=True)
                except Exception as e:
                    print(f"Chatbot: LM Studio on 3090 failed/offline: {e}", flush=True)

                # 2. Fall back to Hugging Face Inference API if 3090 is offline
                if not reply:
                    print("Chatbot: Falling back to Hugging Face Inference API...", flush=True)
                    hf_url = "https://api-inference.huggingface.co/models/Qwen/Qwen2.5-7B-Instruct"
                    headers = {}
                    if hf_token:
                        headers["Authorization"] = f"Bearer {hf_token}"
                    
                    hf_payload = {
                        "inputs": f"<|im_start|>system\n{system_prompt}<|im_end|>\n<|im_start|>user\n{text}<|im_end|>\n<|im_start|>assistant\n",
                        "parameters": {
                            "max_new_tokens": 80,
                            "temperature": 0.7
                        }
                    }
                    try:
                        r = requests.post(hf_url, json=hf_payload, headers=headers, timeout=10.0)
                        if r.status_code == 200:
                            hf_res = r.json()
                            raw_content = ""
                            if isinstance(hf_res, list) and len(hf_res) > 0:
                                generated = hf_res[0].get("generated_text", "")
                                if "assistant\n" in generated:
                                    raw_content = generated.split("assistant\n")[-1].strip()
                                else:
                                    raw_content = generated.strip()
                            elif isinstance(hf_res, dict) and "generated_text" in hf_res:
                                raw_content = hf_res["generated_text"].strip()
                            reply = clean_and_verify_response(raw_content)
                            print(f"Chatbot: Response from HF Inference: '{reply}'", flush=True)
                    except Exception as hf_err:
                        print(f"Chatbot: HF Inference fallback failed: {hf_err}", flush=True)

                # 3. Final default fallback message if everything is down
                if not reply:
                    reply = "Beep boop! I am currently unable to access my artificial brain. Please check the local workstation or internet connection."

                # 4. Trigger speech on Reachy (port 8042)
                try:
                    requests.post("http://127.0.0.1:8042/speak", json={"text": reply}, timeout=5.0)
                except Exception as speak_err:
                    print(f"Chatbot: Failed to forward speech to 127.0.0.1:8042: {speak_err}", flush=True)

                # 5. Return JSON response
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"response": reply}).encode('utf-8'))
                return

            except Exception as e:
                self.send_response(500)
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode('utf-8'))
                return
        else:
            self.send_response(404)
            self.end_headers()

def main():
    # Kill any active processes
    import os
    os.system("pkill -f calibrate_rover.py")
    os.system("pkill -f sbus_listener.py")
    
    # Start serial worker
    threading.Thread(target=serial_worker, daemon=True).start()
    
    # Start ZMQ drive worker (Port 5558)
    threading.Thread(target=zmq_drive_worker, daemon=True).start()
    
    # Start camera workers (0: driving cam, 2: reachy head cam)
    threading.Thread(target=camera_worker, args=(0,), daemon=True).start()
    threading.Thread(target=camera_worker, args=(2,), daemon=True).start()
    
    # Start SO-101 status poller
    threading.Thread(target=so_arm_poller, daemon=True).start()
    
    # Start Reachy status poller
    threading.Thread(target=reachy_daemon_poller, daemon=True).start()
    
    # Start Hot Mic status poller
    threading.Thread(target=hot_mic_poller, daemon=True).start()
    
    # Start web server
    with http.server.ThreadingHTTPServer(("", PORT), RoverHandler) as httpd:
        print(f"Server started at http://localhost:{PORT}")
        print(f"If accessing from another device, use http://192.168.0.130:{PORT}")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("Server shutting down.")

if __name__ == "__main__":
    main()
