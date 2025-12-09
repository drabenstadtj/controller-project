import serial
import serial.tools.list_ports
from pynput.mouse import Controller, Button
import threading
import time
import pyautogui

# ============================================================================
# CONFIGURATION PARAMETERS
# ============================================================================

BAUD_RATE = 230400  # Must match Arduino serial baud rate

# Movement parameters
SENSITIVITY_X = 300    # Pixels from center per 1G of tilt (horizontal)
SENSITIVITY_Y = 300    # Pixels from center per 1G of tilt (vertical)
DEADZONE = 0.2         # Ignore tilts smaller than this (in G's) - prevents drift
MAX_OFFSET = 600       # Maximum cursor distance from screen center (pixels)
SMOOTHING = 0.7        # Exponential smoothing factor (0=none, 1=maximum)

# ============================================================================
# GLOBAL STATE
# ============================================================================

# Calibration offsets (learned during startup)
calibration_offset_x = 0
calibration_offset_y = 0
calibration_offset_z = 0
calibrated = False

# Mouse controller
mouse = Controller()
mouse_enabled = True

# Smoothing state (exponential moving average)
smooth_x = 0.0
smooth_y = 0.0

# Button state tracking
btn1_pressed = False
btn2_pressed = False

# Screen dimensions for centering the virtual joystick
screen_width, screen_height = pyautogui.size()
screen_center_x = screen_width // 2
screen_center_y = screen_height // 2

# ============================================================================
# ARDUINO CONNECTION
# ============================================================================

def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # Look for common Arduino identifiers
        if 'Arduino' in port.description or 'USB Serial' in port.description or 'CH340' in port.description:
            return port.device
    return None

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
time.sleep(2)  # Wait for Arduino to reset after serial connection

# ============================================================================
# MAIN CONTROL LOOP
# ============================================================================

def read_serial():
    """
    Reads data from Arduino and controls the mouse.

    Data Format from Arduino (CSV):
        accelX, accelY, accelZ, button1, button2

    Process:
        1. Calibration Phase: Collect 500 samples to learn neutral position
        2. Mouse Control Phase: Convert tilt to cursor position and handle buttons
    """
    global mouse_enabled, SENSITIVITY_X, SENSITIVITY_Y, DEADZONE
    global smooth_x, smooth_y
    global calibrated, calibration_offset_x, calibration_offset_y, calibration_offset_z
    global MAX_OFFSET, SMOOTHING, screen_center_x, screen_center_y
    global btn1_pressed, btn2_pressed

    # Calibration accumulation variables
    calib_readings_count = 500
    cumulative_x = 0
    cumulative_y = 0
    cumulative_z = 0

    while True:
        try:
            if ser.in_waiting > 0:
                # Read and parse serial line
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                # Skip invalid lines
                if not line or len(line) < 5:
                    continue

                try:
                    accel_match = line.split(',')

                    # Parse data - support both formats for backward compatibility
                    if len(accel_match) == 3:
                        # Old format: just accelerometer data
                        x_val = float(accel_match[0])
                        y_val = float(accel_match[1])
                        z_val = float(accel_match[2])
                        btn1 = 0
                        btn2 = 0
                    elif len(accel_match) == 5:
                        # New format: accelerometer + button data
                        x_val = float(accel_match[0])
                        y_val = float(accel_match[1])
                        z_val = float(accel_match[2])
                        btn1 = int(accel_match[3])
                        btn2 = int(accel_match[4])
                    else:
                        continue

                except (ValueError, IndexError):
                    # Malformed data, skip this reading
                    continue

                # Calibration phase
                # Accumulate samples while controller is at rest to learn the
                # neutral position (accounts for sensor mounting angle and bias)
                if not calibrated:
                    if calib_readings_count > 0:
                        cumulative_x += x_val
                        cumulative_y += y_val
                        cumulative_z += z_val
                        calib_readings_count -= 1

                        if calib_readings_count % 100 == 0:
                            print(f"Calibrating... {calib_readings_count} readings left")
                    else:
                        # Calculate average neutral position
                        calibration_offset_x = cumulative_x / 500
                        calibration_offset_y = cumulative_y / 500
                        calibration_offset_z = cumulative_z / 500
                        calibrated = True
                        print(f"Calibration complete! Offsets: X={calibration_offset_x:.3f}, Y={calibration_offset_y:.3f}, Z={calibration_offset_z:.3f}")
                        print("Mouse control active!")

                # Mouse control phase
                elif mouse_enabled:
                    # Process button state changes BEFORE cursor movement so that
                    # dragging works correctly (button pressed, then cursor moves)

                    # Button 1 - Left mouse button (click/drag)
                    if btn1 == 1 and not btn1_pressed:
                        mouse.press(Button.left)
                        btn1_pressed = True
                    elif btn1 == 0 and btn1_pressed:
                        mouse.release(Button.left)
                        btn1_pressed = False

                    # Button 2 - Right mouse button (click/drag)
                    if btn2 == 1 and not btn2_pressed:
                        mouse.press(Button.right)
                        btn2_pressed = True
                    elif btn2 == 0 and btn2_pressed:
                        mouse.release(Button.right)
                        btn2_pressed = False

                    # Remove calibration offset to get tilt relative to neutral
                    tilt_x = -(x_val - calibration_offset_x)  # Flipped for intuitive control
                    tilt_y = (y_val - calibration_offset_y)

                    # Apply deadzone to prevent drift when controller is nearly neutral
                    #    Small movements below DEADZONE threshold are zeroed out
                    if abs(tilt_x) < DEADZONE:
                        tilt_x = 0
                    else:
                        # Remove deadzone from active range to avoid sudden jump when crossing the zone
                        sign = 1 if tilt_x > 0 else -1
                        tilt_x = sign * (abs(tilt_x) - DEADZONE)

                    if abs(tilt_y) < DEADZONE:
                        tilt_y = 0
                    else:
                        sign = 1 if tilt_y > 0 else -1
                        tilt_y = sign * (abs(tilt_y) - DEADZONE)

                    # Scale tilt to pixel offset from screen center
                    target_x = tilt_x * SENSITIVITY_X
                    target_y = tilt_y * SENSITIVITY_Y

                    # Clamp to maximum offset (circular boundary)
                    #    Prevents cursor from going off-screen with extreme tilts
                    distance = (target_x**2 + target_y**2)**0.5
                    if distance > MAX_OFFSET:
                        scale = MAX_OFFSET / distance
                        target_x *= scale
                        target_y *= scale

                    # Apply exponential smoothing to reduce jitter
                    #    smooth = α * old_smooth + (1-α) * new_target
                    smooth_x = SMOOTHING * smooth_x + (1 - SMOOTHING) * target_x
                    smooth_y = SMOOTHING * smooth_y + (1 - SMOOTHING) * target_y

                    # Convert from offset to absolute screen coordinates
                    target_screen_x = screen_center_x + int(smooth_x)
                    target_screen_y = screen_center_y + int(smooth_y)

                    # Move cursor (will drag if a button is currently pressed)
                    mouse.position = (target_screen_x, target_screen_y)

        except Exception as e:
            print(f"Read error: {e}")
            time.sleep(0.01)

# ============================================================================
# STARTUP
# ============================================================================

# Run serial reading in background thread so main thread can handle Ctrl+C
serial_thread = threading.Thread(target=read_serial, daemon=True)
serial_thread.start()

print("Mouse controller running. Press Ctrl+C to exit.")

# Keep main thread alive
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()
