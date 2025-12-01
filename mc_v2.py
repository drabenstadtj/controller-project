import serial
import serial.tools.list_ports
from pynput.mouse import Controller
import threading
import time
import pyautogui  # For getting screen size

# Auto-detect Arduino port
def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'Arduino' in port.description or 'USB Serial' in port.description or 'CH340' in port.description:
            return port.device
    return None

BAUD_RATE = 230400

# TUNING PARAMETERS FOR GAME CONTROL
SENSITIVITY_X = 300  # How far cursor moves from center per G
SENSITIVITY_Y = 300
DEADZONE = 0.15  # Larger deadzone - hand at rest = cursor at center
MAX_OFFSET = 600  # Maximum pixels from center (joystick radius)
SMOOTHING = 0.3  # Smooth cursor movement (0=raw, 1=very smooth)

calibration_offset_x = 0
calibration_offset_y = 0
calibration_offset_z = 0
calibrated = False

# Use pynput instead of pyautogui
mouse = Controller()

# Smoothing state
smooth_x = 0.0
smooth_y = 0.0
mouse_enabled = True

# Get screen dimensions for centering
screen_width, screen_height = pyautogui.size()
screen_center_x = screen_width // 2
screen_center_y = screen_height // 2

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

def apply_deadzone(value, deadzone):
    """Apply deadzone with smooth transition"""
    if abs(value) < deadzone:
        return 0
    sign = 1 if value > 0 else -1
    magnitude = (abs(value) - deadzone) / (1 - deadzone)
    return sign * magnitude

def read_serial():
    """Thread to continuously read from Arduino"""
    global mouse_enabled, SENSITIVITY_X, SENSITIVITY_Y, DEADZONE
    global smooth_x, smooth_y
    global calibrated, calibration_offset_x, calibration_offset_y, calibration_offset_z
    global MAX_OFFSET, SMOOTHING, screen_center_x, screen_center_y
    
    calib_readings_count = 500  # Reduced for faster calibration
    cumulative_x = 0
    cumulative_y = 0
    cumulative_z = 0
    
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if not line or len(line) < 5:
                    continue
                
                try:
                    accel_match = line.split(',')
                    
                    if len(accel_match) != 3:
                        continue
                    
                    x_val = float(accel_match[0])
                    y_val = float(accel_match[1])
                    z_val = float(accel_match[2])
                    
                except (ValueError, IndexError):
                    continue
                
                # Calibration phase
                if not calibrated:
                    if calib_readings_count > 0:
                        cumulative_x += x_val
                        cumulative_y += y_val
                        cumulative_z += z_val
                        calib_readings_count -= 1
                        
                        if calib_readings_count % 100 == 0:
                            print(f"Calibrating... {calib_readings_count} readings left")
                    else:
                        calibration_offset_x = cumulative_x / 500
                        calibration_offset_y = cumulative_y / 500
                        calibration_offset_z = cumulative_z / 500
                        calibrated = True
                        print(f"Calibration complete! Offsets: X={calibration_offset_x:.3f}, Y={calibration_offset_y:.3f}, Z={calibration_offset_z:.3f}")
                        print("Mouse control active!")
                
                # Mouse control phase - JOYSTICK MODE
                elif mouse_enabled:
                    # Get tilt angles (acceleration values represent tilt)
                    tilt_x = (x_val - calibration_offset_x)
                    tilt_y = (y_val - calibration_offset_y)

                    # Apply deadzone
                    if abs(tilt_x) < DEADZONE:
                        tilt_x = 0
                    else:
                        # Scale after deadzone
                        sign = 1 if tilt_x > 0 else -1
                        tilt_x = sign * (abs(tilt_x) - DEADZONE)

                    if abs(tilt_y) < DEADZONE:
                        tilt_y = 0
                    else:
                        sign = 1 if tilt_y > 0 else -1
                        tilt_y = sign * (abs(tilt_y) - DEADZONE)

                    # Calculate target cursor position relative to screen center
                    target_x = tilt_x * SENSITIVITY_X
                    target_y = tilt_y * SENSITIVITY_Y

                    # Clamp to maximum offset (joystick radius)
                    distance = (target_x**2 + target_y**2)**0.5
                    if distance > MAX_OFFSET:
                        scale = MAX_OFFSET / distance
                        target_x *= scale
                        target_y *= scale

                    # Apply smoothing for stability
                    smooth_x = SMOOTHING * smooth_x + (1 - SMOOTHING) * target_x
                    smooth_y = SMOOTHING * smooth_y + (1 - SMOOTHING) * target_y

                    # Calculate absolute screen position
                    target_screen_x = screen_center_x + int(smooth_x)
                    target_screen_y = screen_center_y + int(smooth_y)

                    # Set cursor to absolute position
                    mouse.position = (target_screen_x, target_screen_y)
                        
        except Exception as e:
            print(f"Read error: {e}")
            time.sleep(0.01)

# Run in thread
serial_thread = threading.Thread(target=read_serial, daemon=True)
serial_thread.start()

print("Mouse controller running. Press Ctrl+C to exit.")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()