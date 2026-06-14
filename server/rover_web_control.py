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

PORT = 8080

# Auto-detect serial port
SERIAL_PORT = "/dev/ttyACM1" if os.path.exists("/dev/ttyACM1") else "/dev/ttyACM0"

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
    "ch5": 1000
}

# Joystick target values (Y = throttle, X = steering)
joystick_x = 0.0
joystick_y = 0.0
last_joystick_update = 0.0

# Camera state
camera_lock = threading.Lock()
latest_frame = None

# Camera capture worker
def camera_worker():
    global latest_frame
    print("Starting camera worker thread...")
    cap = None
    
    while True:
        try:
            if cap is None or not cap.isOpened():
                # Open webcam /dev/video0
                cap = cv2.VideoCapture(0)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                time.sleep(1.0)
                
            if cap.isOpened():
                success, frame = cap.read()
                if success:
                    # Compress to JPEG
                    _, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                    frame_bytes = jpeg.tobytes()
                    with camera_lock:
                        latest_frame = frame_bytes
                else:
                    print("Failed to read camera. Re-initializing...")
                    cap.release()
                    cap = None
            else:
                # Generate "Camera Offline" placeholder frame
                placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.rectangle(placeholder, (15, 15), (625, 465), (20, 20, 30), -1)
                cv2.rectangle(placeholder, (10, 10), (630, 470), (0, 70, 255), 1)
                cv2.putText(placeholder, "CAMERA OFFLINE", (170, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 70, 255), 2)
                _, jpeg = cv2.imencode('.jpg', placeholder)
                with camera_lock:
                    latest_frame = jpeg.tobytes()
                time.sleep(1.0)
                
            time.sleep(0.04) # ~25 FPS
            
        except Exception as e:
            print(f"Camera worker error: {e}")
            if cap:
                cap.release()
                cap = None
            time.sleep(2.0)

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
                    cmd_str = f"CMD:{left_pulse},{right_pulse}\n"
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

        #videoStream {
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

        .stop-button:active {
            transform: scale(0.97);
        }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>Overlander-4 Console</h1>
            <p class="subtitle">Unified Web Operation & Telemetry</p>
        </header>

        <div class="main-grid">
            <!-- Left Side: Video Stream & Telemetry -->
            <div style="display: flex; flex-direction: column; gap: 1.2rem;">
                <div class="card">
                    <h2>Live Video Stream</h2>
                    <div class="video-viewport">
                        <img id="videoStream" src="/video_feed" alt="Video Stream" />
                    </div>
                </div>

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

        function resetHandle() {
            joyHandle.style.left = 'calc(50% - 35px)';
            joyHandle.style.top = 'calc(50% - 35px)';
            joyHandle.style.transform = 'translate(0px, 0px)';
            sendDriveCommand(0, 0);
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
            const xVal = dx / maxLimit;
            const yVal = -dy / maxLimit;
            
            sendDriveCommand(xVal, yVal);
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
        elif self.path == "/video_feed":
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            
            last_frame_sent = None
            try:
                while True:
                    with camera_lock:
                        frame = latest_frame
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
    
    # Start camera worker
    threading.Thread(target=camera_worker, daemon=True).start()
    
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
