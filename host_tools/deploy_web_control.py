import paramiko
import sys

def main():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        print("Connecting to Pi...")
        ssh.connect("192.168.0.130", username="user", password="raspberry", timeout=5)
        
        web_server_code = """import http.server
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

PORT = 8080

# Auto-detect serial port (preferring Adafruit KB2040 by ID)
import glob
kb_ports = glob.glob("/dev/serial/by-id/*Adafruit_KB2040*")
SERIAL_PORT = kb_ports[0] if kb_ports else ("/dev/ttyACM0" if os.path.exists("/dev/ttyACM0") else "/dev/ttyACM1")

def detect_follower_port():
    import os
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
    "flags": 0,
    "so_arm_active": 2, # 0: inactive, 1: active, 2: offline
    "reachy_daemon_active": 2 # 0: inactive, 1: active, 2: offline
}

# Joystick target values (Y = throttle, X = steering)
joystick_x = 0.0
joystick_y = 0.0
last_joystick_update = 0.0

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
        url = "http://localhost:8000/api/camera/frame"
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
                ["pgrep", "-f", "so101_host.py"],
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
    print("Starting Reachy daemon status poller thread...")
    while True:
        try:
            res = subprocess.run(
                ["pgrep", "-f", "reachy-mini-daemon"],
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
            latest_telemetry["reachy_daemon_active"] = status
            
        time.sleep(2.5)

# Serial communication thread
def serial_worker():
    global latest_telemetry
    print(f"Opening serial port {SERIAL_PORT}...")
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, 115200, timeout=1.0)
            ser.reset_input_buffer()
            last_send_time = 0.0
            
            while True:
                now = time.time()
                
                # 1. Read telemetry line from KB2040
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
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
                                latest_telemetry["flags"] = int(parts[8]) if len(parts) > 8 else 0
                        except Exception as e:
                            pass
                
                # 2. Send driving commands (heartbeat every 100ms)
                if (now - last_send_time) > 0.10:
                    last_send_time = now
                    with lock:
                        x = joystick_x
                        y = joystick_y
                        update_age = now - last_joystick_update
                    
                    # Failsafe: if no update from web client, stop motors
                    if update_age > 1.0:
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
                    
                    # Map to pulse widths (1100 to 1900 microseconds, center 1500)
                    # Note: Left motor is physically inverted (higher width = reverse), and Right motor is normal (higher width = forward).
                    left_pulse = 1500 - int(left * 400)
                    right_pulse = 1500 + int(right * 400)
                    
                    # Write command to KB2040
                    cmd_str = f"CMD:{left_pulse},{right_pulse}\\n"
                    ser.write(cmd_str.encode('utf-8'))
                    if x != 0.0 or y != 0.0:
                        print(f"JOYSTICK: x={x:.3f}, y={y:.3f} -> throttle={throttle:.3f}, steering={steering:.3f} -> left={left_pulse}, right={right_pulse}", flush=True)
                    
                time.sleep(0.01)
                
        except Exception as e:
            print(f"Serial error: {e}. Retrying in 2 seconds...")
            time.sleep(2.0)

# HTML Page serving the Web Controller UI
HTML_PAGE = \"\"\"<!DOCTYPE html>
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
            width: 70px;
            height: 70px;
            background: linear-gradient(135deg, var(--primary), var(--secondary));
            border-radius: 50%;
            position: absolute;
            cursor: pointer;
            box-shadow: 0 8px 16px rgba(0, 242, 254, 0.3), 0 0 10px rgba(255,255,255,0.2);
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

        // Center the handle initially
        resetHandle();

        let joystickX = 0.0;
        let joystickY = 0.0;

        function resetHandle() {
            joyHandle.style.left = 'calc(50% - 35px)';
            joyHandle.style.top = 'calc(50% - 35px)';
            joyHandle.style.transform = 'translate(0px, 0px)';
            joystickX = 0.0;
            joystickY = 0.0;
        }

        function handleStart(e) {
            isDragging = true;
            const clientX = e.touches ? e.touches[0].clientX : e.clientX;
            const clientY = e.touches ? e.touches[0].clientY : e.clientY;
            
            const rect = joyBase.getBoundingClientRect();
            startX = rect.left + rect.width / 2;
            startY = rect.top + rect.height / 2;
            
            handleMove(e);
        }

        function handleMove(e) {
            if (!isDragging) return;
            e.preventDefault();
            
            const clientX = e.touches ? e.touches[0].clientX : e.clientX;
            const clientY = e.touches ? e.touches[0].clientY : e.clientY;
            
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

        function handleEnd() {
            if (!isDragging) return;
            isDragging = false;
            resetHandle();
        }

        // Mouse Events
        joyBase.addEventListener('mousedown', handleStart);
        window.addEventListener('mousemove', handleMove);
        window.addEventListener('mouseup', handleEnd);

        // Touch Events
        joyBase.addEventListener('touchstart', handleStart);
        window.addEventListener('touchmove', handleMove);
        window.addEventListener('touchend', handleEnd);

        // Periodic transmission loop (every 100ms) to prevent server-side failsafe cutouts
        setInterval(() => {
            sendDriveCommand(joystickX, joystickY);
        }, 100);

        // Send Command to Server
        async function sendDriveCommand(x, y) {
            try {
                await fetch('/api/drive', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                    body: `x=${x.toFixed(3)}&y=${y.toFixed(3)}`
                });
            } catch (err) {
                console.error("Failed to send drive cmd", err);
            }
        }

        function emergencyStop() {
            resetHandle();
        }

        async function controlSoArm(action) {
            try {
                const armBadge = document.getElementById('armStatusVal');
                armBadge.textContent = action === "start" ? "STARTING..." : "STOPPING...";
                
                await fetch('/api/so_arm', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                    body: `action=${action}`
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
    </script>
</body>
</html>
\"\"\"

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
            self.end_headers()
            
            last_frame_sent = None
            try:
                while True:
                    with camera_locks[device_idx]:
                        frame = latest_frames[device_idx]
                    if frame and frame != last_frame_sent:
                        self.wfile.write(b'--frame\\r\\n')
                        self.wfile.write(b'Content-Type: image/jpeg\\r\\n\\r\\n')
                        self.wfile.write(frame)
                        self.wfile.write(b'\\r\\n')
                        last_frame_sent = frame
                    time.sleep(0.04) # ~25 FPS
            except Exception as e:
                # Client disconnected
                pass
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == "/api/drive":
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length).decode('utf-8')
            params = urllib.parse.parse_qs(post_data)
            
            if "x" in params and "y" in params:
                global joystick_x, joystick_y, last_joystick_update
                try:
                    x = float(params["x"][0])
                    y = float(params["y"][0])
                    with lock:
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
        elif self.path == "/api/so_arm":
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length).decode('utf-8')
            params = urllib.parse.parse_qs(post_data)
            
            if "action" in params:
                action = params["action"][0]
                import subprocess
                try:
                    if action == "start":
                        subprocess.run(["pkill", "-f", "so101_host.py"], timeout=3.0)
                        port = detect_follower_port()
                        cmd = [
                            "/home/user/so101/.venv/bin/python",
                            "/home/user/so101/so101_host.py",
                            "--port", port,
                            "--id", "follower"
                        ]
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
                        subprocess.run(["pkill", "-f", "so101_host.py"], timeout=3.0)
                        
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
                        subprocess.run(["pkill", "-f", "reachy-mini-daemon"], timeout=3.0)
                        # Brief wait for OS to release ports
                        time.sleep(1.0)
                        cmd = [
                            "/home/user/reachy/venv/bin/reachy-mini-daemon",
                            "-p", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AAF262436-if00"
                        ]
                        log_file = open("/home/user/reachy_daemon.log", "w")
                        subprocess.Popen(
                            cmd,
                            stdout=log_file,
                            stderr=log_file,
                            stdin=subprocess.DEVNULL,
                            start_new_session=True,
                            close_fds=True
                        )
                    elif action == "stop":
                        subprocess.run(["pkill", "-f", "reachy-mini-daemon"], timeout=3.0)
                        
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
    
    # Start camera workers (0: driving cam, 2: reachy head cam)
    threading.Thread(target=camera_worker, args=(0,), daemon=True).start()
    threading.Thread(target=camera_worker, args=(2,), daemon=True).start()
    
    # Start SO-101 status poller
    threading.Thread(target=so_arm_poller, daemon=True).start()
    
    # Start Reachy status poller
    threading.Thread(target=reachy_daemon_poller, daemon=True).start()
    
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
"""
        
        sftp = ssh.open_sftp()
        with sftp.open("/home/user/rover_web_control.py", "w") as f:
            f.write(web_server_code)
        sftp.close()
        
        print("Web controller code written successfully!")
        
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        ssh.close()

if __name__ == "__main__":
    main()
