import serial
import serial.tools.list_ports
import pyautogui
import re
import time
import threading
import sys

# Auto-detect Arduino port
def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'Arduino' in port.description or 'USB Serial' in port.description or 'CH340' in port.description:
            return port.device
    return None

BAUD_RATE = 115200

# TUNING PARAMETERS
SENSITIVITY_X = 150        # Side-to-side sensitivity (increased)
SENSITIVITY_Y = 150        # Up-down sensitivity
DEADZONE = 0.20           # Lower deadzone for better responsiveness
SMOOTHING_ALPHA = 0.15    # Lower = smoother (0.1-0.3 range)
MIN_MOVEMENT = 1          # Minimum pixels to move (reduces micro-jitter)

pyautogui.FAILSAFE = False

# Smoothing state
smooth_vel_x = 0.0
smooth_vel_y = 0.0
accumulated_x = 0.0
accumulated_y = 0.0

print("Searching for Arduino...")
SERIAL_PORT = find_arduino_port()

if SERIAL_PORT is None:
    print("Arduino not found! Available ports:")
    for port in serial.tools.list_ports.comports():
        print(f"  {port.device} - {port.description}")
    exit(1)

print(f"Found Arduino on {SERIAL_PORT}")
print("Connecting...")

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

print("\n" + "="*50)
print("Connected! 3D Motion Controller - SMOOTH v2")
print("="*50)
print("\nCommands:")
print("  c - Calibrate (HOLD VERY STILL)")
print("  s - Start/Stop tracking")
print("  r - Reset")
print("  x+ / x- - Adjust X sensitivity (side-to-side)")
print("  y+ / y- - Adjust Y sensitivity (up-down)")
print("  d / D - Adjust deadzone")
print("  q - Quit\n")

mouse_enabled = False
last_move_time = time.time()

def apply_deadzone(value, deadzone):
    """Apply deadzone with smooth transition"""
    if abs(value) < deadzone:
        return 0
    # Smooth scaling after deadzone
    sign = 1 if value > 0 else -1
    magnitude = (abs(value) - deadzone) / (1 - deadzone)
    return sign * magnitude

def read_serial():
    """Thread to continuously read from Arduino"""
    global mouse_enabled, SENSITIVITY_X, SENSITIVITY_Y, DEADZONE
    global smooth_vel_x, smooth_vel_y, accumulated_x, accumulated_y, last_move_time
    
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                
                if 'VEL:' in line and mouse_enabled:
                    vel_match = re.search(r'VEL:([-\d.]+),([-\d.]+),([-\d.]+)', line)
                    if vel_match:
                        velX = float(vel_match.group(1))
                        velY = float(vel_match.group(2))
                        
                        # Apply deadzone
                        velX = apply_deadzone(velX, DEADZONE)
                        velY = apply_deadzone(velY, DEADZONE)
                        
                        # Exponential smoothing (this removes judder)
                        smooth_vel_x = SMOOTHING_ALPHA * velX + (1 - SMOOTHING_ALPHA) * smooth_vel_x
                        smooth_vel_y = SMOOTHING_ALPHA * velY + (1 - SMOOTHING_ALPHA) * smooth_vel_y
                        
                        # Calculate time delta for consistent speed
                        current_time = time.time()
                        dt = current_time - last_move_time
                        last_move_time = current_time
                        
                        # Apply different sensitivities for X and Y
                        move_x = smooth_vel_x * SENSITIVITY_X * dt
                        move_y = smooth_vel_y * SENSITIVITY_Y * dt
                        
                        # Accumulate sub-pixel movements
                        accumulated_x += move_x
                        accumulated_y += move_y
                        
                        # Only move when we have at least MIN_MOVEMENT pixels
                        pixel_x = 0
                        pixel_y = 0
                        
                        if abs(accumulated_x) >= MIN_MOVEMENT:
                            pixel_x = int(accumulated_x)
                            accumulated_x -= pixel_x
                        
                        if abs(accumulated_y) >= MIN_MOVEMENT:
                            pixel_y = int(accumulated_y)
                            accumulated_y -= pixel_y
                        
                        # Move the mouse
                        if pixel_x != 0 or pixel_y != 0:
                            pyautogui.moveRel(pixel_x, pixel_y)
                            
                else:
                    print(line)
        except Exception as e:
            print(f"Read error: {e}")
            time.sleep(0.1)

def send_command():
    """Main thread to handle user input"""
    global mouse_enabled, SENSITIVITY_X, SENSITIVITY_Y, DEADZONE
    global smooth_vel_x, smooth_vel_y, accumulated_x, accumulated_y
    
    while True:
        try:
            cmd = input(">>> ").strip().lower()
            
            if cmd == 'q':
                print("Exiting...")
                ser.close()
                sys.exit(0)
                
            elif cmd == 'x+':
                SENSITIVITY_X += 10
                print(f"X Sensitivity (side-to-side): {SENSITIVITY_X}")
                
            elif cmd == 'x-':
                SENSITIVITY_X = max(5, SENSITIVITY_X - 10)
                print(f"X Sensitivity (side-to-side): {SENSITIVITY_X}")
                
            elif cmd == 'y+':
                SENSITIVITY_Y += 10
                print(f"Y Sensitivity (up-down): {SENSITIVITY_Y}")
                
            elif cmd == 'y-':
                SENSITIVITY_Y = max(5, SENSITIVITY_Y - 10)
                print(f"Y Sensitivity (up-down): {SENSITIVITY_Y}")
                
            elif cmd == 'd':
                DEADZONE += 0.05
                print(f"Deadzone: {DEADZONE:.2f} (less drift)")
                
            elif cmd.upper() == 'D':
                DEADZONE = max(0, DEADZONE - 0.05)
                print(f"Deadzone: {DEADZONE:.2f} (more responsive)")
                
            elif cmd in ['c', 's', 'r']:
                ser.write(cmd.encode())
                if cmd == 's':
                    mouse_enabled = not mouse_enabled
                    # Reset smoothing
                    smooth_vel_x = 0
                    smooth_vel_y = 0
                    accumulated_x = 0
                    accumulated_y = 0
                    if mouse_enabled:
                        print(">>> Mouse control ENABLED")
                        print(f"    X-Sensitivity: {SENSITIVITY_X}, Y-Sensitivity: {SENSITIVITY_Y}")
                        print(f"    Deadzone: {DEADZONE:.2f}")
                    else:
                        print(">>> Mouse control DISABLED")
                time.sleep(0.1)
            else:
                print("Commands: c, s, r, x+, x-, y+, y-, d, D, q")
                
        except KeyboardInterrupt:
            print("\nExiting...")
            ser.close()
            sys.exit(0)
        except Exception as e:
            print(f"Command error: {e}")

# Start serial reading in background thread
serial_thread = threading.Thread(target=read_serial, daemon=True)
serial_thread.start()

# Handle commands in main thread
try:
    send_command()
except Exception as e:
    print(f"Error: {e}")
    ser.close()