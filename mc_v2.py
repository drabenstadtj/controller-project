import serial
import serial.tools.list_ports
import pyautogui
import re
import time

# Auto-detect Arduino port
def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'Arduino' in port.description or 'USB Serial' in port.description or 'CH340' in port.description:
            return port.device
    return None

BAUD_RATE = 115200

# TUNING PARAMETERS
SENSITIVITY_X = 200        # Side-to-side sensitivity (increased)
SENSITIVITY_Y = 200       # Up-down sensitivity
DEADZONE = 0           # Lower deadzone for better responsiveness
SMOOTHING_ALPHA = 0.3    # Lower = smoother (0.1-0.3 range)
MIN_MOVEMENT = 1          # Minimum pixels to move (reduces micro-jitter)

calibration_offset_x = 0
calibration_offset_y = 0
calibration_offset_z = 0

calibrated = False

pyautogui.FAILSAFE = False

# Smoothing state
smooth_vel_x = 0.0
smooth_vel_y = 0.0
accumulated_x = 0.0
accumulated_y = 0.0

last_time = 0

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

mouse_enabled = True
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
    global calibrated, calibration_offset_x, calibration_offset_y, calibration_offset_z
    
    calib_readings_count = 1000
    cumulative_x = 0
    cumulative_y = 0
    cumulative_z = 0
    
    while True:
        try:
            if ser.in_waiting > 0:
                # Use errors='ignore' to skip bad bytes
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Skip empty lines or invalid data
                if not line or len(line) < 5:
                    continue
                
                # Try to parse the data
                try:
                    accel_match = line.split(',')
                    
                    # Validate we have 3 values
                    if len(accel_match) != 3:
                        continue
                    
                    x_val = float(accel_match[0])
                    y_val = float(accel_match[1])
                    z_val = float(accel_match[2])
                    
                except (ValueError, IndexError):
                    # Skip malformed data
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
                        calibration_offset_x = cumulative_x / 1000
                        calibration_offset_y = cumulative_y / 1000
                        calibration_offset_z = cumulative_z / 1000
                        calibrated = True
                        print(f"Calibration complete! Offsets: X={calibration_offset_x:.3f}, Y={calibration_offset_y:.3f}, Z={calibration_offset_z:.3f}")
                        print("Mouse control active!")
                
                # Mouse control phase
                elif mouse_enabled:
                    current_time = time.time()
                    delta_time = (current_time - last_move_time)
                    last_move_time = current_time
                    
                    accelX = x_val - calibration_offset_x
                    accelY = y_val - calibration_offset_y
                    
                    velX = accelX * delta_time
                    velY = accelY * delta_time
                    
                    # Apply deadzone
                    velX = apply_deadzone(velX, DEADZONE)
                    velY = apply_deadzone(velY, DEADZONE)
                    
                    # Exponential smoothing
                    smooth_vel_x = SMOOTHING_ALPHA * velX + (1 - SMOOTHING_ALPHA) * smooth_vel_x
                    smooth_vel_y = SMOOTHING_ALPHA * velY + (1 - SMOOTHING_ALPHA) * smooth_vel_y
                    
                    # Apply sensitivities
                    move_x = smooth_vel_x * SENSITIVITY_X * delta_time
                    move_y = smooth_vel_y * SENSITIVITY_Y * delta_time
                    
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
                        pyautogui.moveRel(pixel_x, pixel_y, 0)
                        
        except Exception as e:
            print(f"Read error: {e}")
            time.sleep(0.1)

read_serial()