import http.server
import socketserver
import urllib.parse
import json
import subprocess
import os
import time
import threading
import socket
import urllib.request

PORT = 8082
DIRECTORY = "/home/carson/touch_ui"
PI500_IP = "192.168.0.130"
PI500_HOST = "user@192.168.0.130"
MAC_HOST = "twinpeakstownie@192.168.0.243"

STATUS_CACHE = {
    "pokeball": {"running": False, "pid": ""},
    "follower": {"running": False, "pid": ""},
    "leader": {"running": False, "pid": ""},
    "pi500_online": False
}

def poll_status_loop():
    while True:
        # Check Pi 500 online via fast TCP socket connect (10ms)
        try:
            s = socket.create_connection((PI500_IP, 8085), timeout=0.5)
            s.close()
            STATUS_CACHE["pi500_online"] = True
        except Exception:
            try:
                res = subprocess.run(["ping", "-c", "1", "-W", "1", PI500_IP], capture_output=True)
                STATUS_CACHE["pi500_online"] = (res.returncode == 0)
            except Exception:
                STATUS_CACHE["pi500_online"] = False

        # 1. Check Pokeball
        try:
            res = subprocess.run(
                ["ssh", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=2", PI500_HOST, "pgrep -f pokeball_teleop_driver.py"],
                capture_output=True, text=True, timeout=3.0
            )
            if res.returncode == 0 and res.stdout.strip():
                STATUS_CACHE["pokeball"] = {"running": True, "pid": res.stdout.strip().split()[0]}
            else:
                STATUS_CACHE["pokeball"] = {"running": False, "pid": ""}
        except Exception:
            STATUS_CACHE["pokeball"] = {"running": False, "pid": ""}

        # 2. Check Pi500 Follower
        try:
            res = subprocess.run(
                ["ssh", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=2", PI500_HOST, "pgrep -f sewer_daemon.py"],
                capture_output=True, text=True, timeout=3.0
            )
            if res.returncode == 0 and res.stdout.strip():
                STATUS_CACHE["follower"] = {"running": True, "pid": res.stdout.strip().split()[0]}
            else:
                STATUS_CACHE["follower"] = {"running": False, "pid": ""}
        except Exception:
            STATUS_CACHE["follower"] = {"running": False, "pid": ""}

        # 3. Check Mac Leader
        try:
            res = subprocess.run(
                ["ssh", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=2", MAC_HOST, "pgrep -f so101_leader_client.py"],
                capture_output=True, text=True, timeout=3.0
            )
            if res.returncode == 0 and res.stdout.strip():
                STATUS_CACHE["leader"] = {"running": True, "pid": res.stdout.strip().split()[0]}
            else:
                STATUS_CACHE["leader"] = {"running": False, "pid": ""}
        except Exception:
            STATUS_CACHE["leader"] = {"running": False, "pid": ""}

        time.sleep(2.0)

threading.Thread(target=poll_status_loop, daemon=True).start()

class CustomHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path.rstrip('/')

        if path in ["/api/status", "/api/pokeball_teleop_status"]:
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            data = STATUS_CACHE["pokeball"].copy()
            data["pi500_online"] = STATUS_CACHE["pi500_online"]
            data["active"] = data["running"]
            self.wfile.write(json.dumps(data).encode())
            return

        if path == "/api/pi500_follower_status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(STATUS_CACHE["follower"]).encode())
            return

        if path == "/api/mac_leader_status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(STATUS_CACHE["leader"]).encode())
            return

        if path == "/api/wifi_scan":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            networks = []
            try:
                res = subprocess.run(["nmcli", "-t", "-f", "SSID,SIGNAL", "dev", "wifi"], capture_output=True, text=True, timeout=3.0)
                if res.returncode == 0:
                    for line in res.stdout.strip().split("\n"):
                        if ":" in line:
                            ssid, sig = line.split(":", 1)
                            if ssid:
                                networks.append({"ssid": ssid, "signal": sig})
            except Exception:
                pass
            self.wfile.write(json.dumps({"networks": networks[:4]}).encode())
            return

        super().do_GET()

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path.rstrip('/')

        length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(length).decode('utf-8') if length > 0 else '{}'
        try:
            req_data = json.loads(body)
        except Exception:
            req_data = {}

        if path == "/api/pi500_poweron":
            print("⚡ Triggering Layer-2 Ethernet Wake-on-LAN poweron packet to Pi 500...", flush=True)
            dispatched = False
            try:
                import socket
                mac_hex = "d83add8a4642"
                mac_bytes = bytes.fromhex(mac_hex)
                magic_payload = b'\xff' * 6 + (mac_bytes * 16)
                
                # 1. Try raw Layer-2 socket bound directly to eth0 interface
                try:
                    raw_sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0842))
                    raw_sock.bind(("eth0", 0))
                    # Frame: Dst MAC (FF:FF:FF:FF:FF:FF) + Src MAC (00:00:00:00:00:00) + EtherType (0x0842) + Payload
                    eth_frame = b'\xff' * 6 + b'\x00' * 6 + b'\x08\x42' + magic_payload
                    raw_sock.send(eth_frame)
                    raw_sock.close()
                    dispatched = True
                    print("  - Sent Raw Layer-2 Magic Packet out eth0 interface.", flush=True)
                except Exception as e1:
                    print(f"  - Raw socket attempt: {e1}", flush=True)

                # 2. Try UDP Broadcast on eth0 / wlan0
                try:
                    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                    try:
                        udp_sock.setsockopt(socket.SOL_SOCKET, 25, b'eth0\x00') # SO_BINDTODEVICE
                    except Exception:
                        pass
                    udp_sock.sendto(magic_payload, ('255.255.255.255', 9))
                    udp_sock.sendto(magic_payload, ('192.168.0.255', 9))
                    udp_sock.close()
                    dispatched = True
                except Exception as e2:
                    print(f"  - UDP broadcast attempt: {e2}", flush=True)

                # 3. Call system WOL utilities
                subprocess.Popen(["wakeonlan", "-i", "eth0", "d8:3a:dd:8a:46:42"])
                subprocess.Popen(["wakeonlan", "d8:3a:dd:8a:46:42"])
                subprocess.Popen(["etherwake", "-i", "eth0", "d8:3a:dd:8a:46:42"])
            except Exception as e:
                print(f"WOL overall error: {e}", flush=True)

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok", "message": "Layer-2 WOL magic packet dispatched"}).encode())
            return

        if path in ["/api/toggle", "/api/pokeball_teleop_toggle"]:
            action = req_data.get("action", "toggle")
            if action == "start":
                subprocess.Popen(["ssh", "-o", "StrictHostKeyChecking=no", PI500_HOST, "nohup python3 /home/user/pokeball_teleop_driver.py > /tmp/pokeball.log 2>&1 &"])
            elif action == "stop":
                subprocess.run(["ssh", "-o", "StrictHostKeyChecking=no", PI500_HOST, "pkill -f pokeball_teleop_driver.py"])

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok", "action": action}).encode())
            return

        if path == "/api/pi500_follower_toggle":
            action = req_data.get("action", "toggle")
            if action == "start":
                subprocess.Popen(["ssh", "-o", "StrictHostKeyChecking=no", PI500_HOST, "nohup /home/user/so101/.venv/bin/python /home/user/sewer_daemon.py > /tmp/follower.log 2>&1 &"])
            elif action == "stop":
                subprocess.run(["ssh", "-o", "StrictHostKeyChecking=no", PI500_HOST, "pkill -f sewer_daemon.py"])

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok", "action": action}).encode())
            return

        if path == "/api/mac_leader_toggle":
            action = req_data.get("action", "toggle")
            if action == "start":
                subprocess.Popen(["ssh", "-o", "StrictHostKeyChecking=no", MAC_HOST, "nohup /Users/twinpeakstownie/lerobot/.venv/bin/python /Users/twinpeakstownie/lerobot/so101_leader_client.py > /tmp/leader.log 2>&1 &"])
            elif action == "stop":
                subprocess.run(["ssh", "-o", "StrictHostKeyChecking=no", MAC_HOST, "pkill -f so101_leader_client.py"])

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok", "action": action}).encode())
            return

        if path in ["/api/slider", "/api/pedestal"]:
            val = req_data.get("value", 2503 if "slider" in path else 2048)
            try:
                url = "http://192.168.0.130:8085/api/move"
                if "slider" in path:
                    payload = {"id": 8, "target": int(val)}
                else:
                    payload = {"id": 7, "target": int(val)}
                data_bytes = json.dumps(payload).encode('utf-8')
                req = urllib.request.Request(url, data=data_bytes, headers={'Content-Type': 'application/json'})
                urllib.request.urlopen(req, timeout=1.5)
            except Exception as e:
                pass

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok", "value": val}).encode())
            return

        self.send_response(404)
        self.end_headers()

class ReuseTCPServer(socketserver.TCPServer):
    allow_reuse_address = True

with ReuseTCPServer(("", PORT), CustomHandler) as httpd:
    httpd.serve_forever()
