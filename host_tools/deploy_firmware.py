import paramiko
import sys

def main():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        print("Connecting to Pi...")
        ssh.connect("192.168.0.130", username="user", password="raspberry", timeout=5)
        
        firmware_code = """import board
import rp2pio
import array
import time
import digitalio
import neopixel_write
import pwmio
import supervisor
import sys

# NeoPixel setup
pixel_pin = digitalio.DigitalInOut(board.NEOPIXEL)
pixel_pin.direction = digitalio.Direction.OUTPUT

def set_pixel(color):
    g, r, b = color[1], color[0], color[2]
    neopixel_write.neopixel_write(pixel_pin, bytearray([g, r, b]))

# PWM setup at 50Hz for ESCs
# pin 4 is board.D4 (Motor 1), pin 7 is board.D7 (Motor 2)
motor1_pwm = pwmio.PWMOut(board.D4, frequency=50, duty_cycle=0)
motor2_pwm = pwmio.PWMOut(board.D7, frequency=50, duty_cycle=0)

def set_pulse_width(pwm, microseconds):
    microseconds = max(1000, min(2000, microseconds))
    duty = int((microseconds / 20000.0) * 65535)
    pwm.duty_cycle = duty

# PIO program for UART RX (inverted)
program_data = array.array("H", [0x20A0, 0xEA27, 0x4001, 0x0642, 0x2020])

# Instantiate StateMachine for S.BUS on pin 3 (board.D3)
sm = rp2pio.StateMachine(
    program_data,
    frequency=800000,
    first_in_pin=board.D3,
    in_pin_count=1,
    auto_push=True,
    push_threshold=8,
    in_shift_right=True,
)

def decode_sbus(packet):
    if len(packet) < 25:
        return None
    if packet[0] != 0x0F:
        return None
        
    channels = [0] * 16
    channels[0]  = ((packet[1]       | packet[2] << 8) & 0x07FF)
    channels[1]  = ((packet[2] >> 3  | packet[3] << 5) & 0x07FF)
    channels[2]  = ((packet[3] >> 6  | packet[4] << 2 | packet[5] << 10) & 0x07FF)
    channels[3]  = ((packet[5] >> 1  | packet[6] << 7) & 0x07FF)
    channels[4]  = ((packet[6] >> 4  | packet[7] << 4) & 0x07FF)
    channels[5]  = ((packet[7] >> 7  | packet[8] << 1 | packet[9] << 9) & 0x07FF)
    channels[6]  = ((packet[9] >> 2  | packet[10] << 6) & 0x07FF)
    channels[7]  = ((packet[10] >> 5 | packet[11] << 3) & 0x07FF)
    channels[8]  = ((packet[12]      | packet[13] << 8) & 0x07FF)
    channels[9]  = ((packet[13] >> 3 | packet[14] << 5) & 0x07FF)
    channels[10] = ((packet[14] >> 6 | packet[15] << 2 | packet[16] << 10) & 0x07FF)
    channels[11] = ((packet[15] >> 1 | packet[16] << 7) & 0x07FF)
    channels[12] = ((packet[16] >> 4 | packet[17] << 4) & 0x07FF)
    channels[13] = ((packet[17] >> 7 | packet[18] << 1 | packet[19] << 9) & 0x07FF)
    channels[14] = ((packet[19] >> 2 | packet[20] << 6) & 0x07FF)
    channels[15] = ((packet[20] >> 5 | packet[21] << 3) & 0x07FF)
    
    flags = packet[23]
    fail_safe = bool(flags & 0x08)
    frame_lost = bool(flags & 0x04)
    
    return channels, fail_safe, frame_lost

# State variables
packet_buf = bytearray()
last_valid_packet_time = 0.0

latest_channels = [1000] * 16
sbus_active = False
failsafe_active = False
failsafe_count = 0
last_flags = 0

web_left_pulse = 1500
web_right_pulse = 1500
last_web_command_time = 0.0

buf = bytearray(1)
last_print_time = 0.0

while True:
    now = time.monotonic()
    
    # 1. Read S.BUS serial data from StateMachine into our sliding window buffer
    while sm.in_waiting > 0:
        sm.readinto(buf)
        raw_val = buf[0] ^ 0xFF
        packet_buf.append(raw_val)
        
    # Cap packet buffer size to prevent memory leaks if disconnected
    if len(packet_buf) > 100:
        packet_buf = packet_buf[-50:]
        
    # 2. Process sliding window to find valid S.BUS packets
    while len(packet_buf) >= 25:
        if packet_buf[0] == 0x0F:
            # Candidate packet
            packet = packet_buf[:25]
            end_byte = packet[24]
            # S.BUS end bytes must match standard Futaba S.BUS1 or S.BUS2 values
            if end_byte in (0x00, 0x04, 0x14, 0x24, 0x34):
                res = decode_sbus(packet)
                if res:
                    channels, fs, fl = res
                    latest_channels = channels
                    last_valid_packet_time = now
                    last_flags = packet[23]
                    if fs:
                        failsafe_count += 1
                    else:
                        failsafe_count = 0
                    
                    # Require 10 consecutive failsafe frames (~150ms) to trigger failsafe mode
                    if failsafe_count >= 10:
                        failsafe_active = True
                    else:
                        failsafe_active = False
                packet_buf = packet_buf[25:]
            else:
                # Invalid end byte, discard start byte and slide
                packet_buf = packet_buf[1:]
        else:
            # Not a start byte, discard and slide
            packet_buf = packet_buf[1:]
            
    # 3. Check S.BUS status
    sbus_active = (now - last_valid_packet_time) < 0.5
    
    # 4. Read Web Serial commands
    if supervisor.runtime.serial_bytes_available:
        try:
            line = sys.stdin.readline().strip()
            if line.startswith("CMD:"):
                parts = line.split(":")
                vals = parts[1].split(",")
                web_left_pulse = int(vals[0])
                web_right_pulse = int(vals[1])
                last_web_command_time = now
        except Exception:
            pass
            
    web_active = (now - last_web_command_time) < 0.5
    
    # 5. Mode Determination
    if sbus_active:
        if failsafe_active:
            mode = "FAILSAFE"
        elif latest_channels[4] > 1000:
            mode = "RC"
        else:
            mode = "WEB"
    else:
        mode = "WEB" # Default to Web mode if RC is off
        
    # 6. Output calculation
    left_out = 1500
    right_out = 1500
    
    if mode == "RC" and sbus_active and not failsafe_active:
        # Map Channel 1 and Channel 2 (indices 0 and 1)
        ch1 = max(172, min(1811, latest_channels[0]))
        ch2 = max(172, min(1811, latest_channels[1]))
        left_out = 1500 + int((ch1 - 992) * 400 / 820)
        right_out = 1500 + int((ch2 - 992) * 400 / 820)
    elif mode == "WEB":
        if web_active:
            left_out = web_left_pulse
            right_out = web_right_pulse
        else:
            left_out = 1500
            right_out = 1500
            
    # 7. Apply PWM outputs
    set_pulse_width(motor1_pwm, left_out)
    set_pulse_width(motor2_pwm, right_out)
    
    # 8. Status LED (NeoPixel)
    if sbus_active:
        if failsafe_active:
            # Solid Orange: S.BUS receiving but failsafe active
            set_pixel((255, 100, 0))
        elif mode == "RC":
            # Solid Blue: S.BUS receiving, transmitter on, RC mode
            set_pixel((0, 0, 255))
        else:
            if web_active:
                # Solid Green: S.BUS receiving, transmitter on, WEB mode active
                set_pixel((0, 255, 0))
            else:
                # Flashing Green: S.BUS receiving, transmitter on, WEB mode idle
                if int(now * 2) % 2 == 0:
                    set_pixel((0, 255, 0))
                else:
                    set_pixel((0, 0, 0))
    else:
        # Flashing Red: No S.BUS signal at all
        if int(now * 4) % 2 == 0:
            set_pixel((255, 0, 0))
        else:
            set_pixel((0, 0, 0))
            
    # 9. Telemetry Printout (every 100ms)
    if (now - last_print_time) > 0.10:
        last_print_time = now
        print(f"STAT:{mode},{left_out},{right_out},{1 if sbus_active else 0},{1 if web_active else 0},{latest_channels[0]},{latest_channels[1]},{latest_channels[4]},{last_flags}")
"""
        
        sftp = ssh.open_sftp()
        with sftp.open("/media/user/CIRCUITPY/code.py", "w") as f:
            f.write(firmware_code)
        sftp.close()
        
        print("Flushing cache (sync)...")
        stdin, stdout, stderr = ssh.exec_command("sync")
        stdout.channel.recv_exit_status()
        
        print("Firmware deployed to pin 3 successfully with sliding window sync!")
        
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        ssh.close()

if __name__ == "__main__":
    main()
