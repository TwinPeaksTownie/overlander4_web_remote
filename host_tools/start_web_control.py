import paramiko
import time
import sys

def main():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        print("Connecting to Pi...")
        ssh.connect("192.168.0.130", username="user", password="raspberry", timeout=5)
        
        # Kill any existing web control servers first
        print("Stopping any running control servers...")
        ssh.exec_command("pkill -f rover_web_control.py")
        time.sleep(0.5)
        
        # Launch in background
        print("Starting rover_web_control.py in background...")
        ssh.exec_command("nohup python3 /home/user/rover_web_control.py > /tmp/rover_web.log 2>&1 &")
        time.sleep(1.0)
        
        # Check if it is running
        stdin, stdout, stderr = ssh.exec_command("ps aux | grep rover_web_control.py | grep -v grep")
        ps_out = stdout.read().decode('utf-8')
        if ps_out:
            print("Web server is running in the background:")
            print(ps_out.strip())
        else:
            print("Error: Web server failed to start.")
            
        print("\n--- Reading Server Logs (/tmp/rover_web.log) ---")
        stdin, stdout, stderr = ssh.exec_command("cat /tmp/rover_web.log")
        print(stdout.read().decode('utf-8'))
        
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        ssh.close()

if __name__ == "__main__":
    main()
