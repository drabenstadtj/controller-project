# 3D Motion Mouse Controller

A joystick-style mouse controller using an MPU6050 accelerometer sensor connected to an Arduino. Tilt the sensor to move the cursor from the screen center, return to neutral to center the cursor.

## Features

- **Joystick-style control**: Cursor position maps to sensor tilt (not movement)
- **Two button support**: Left/right click and drag operations
- **Automatic calibration**: Learns neutral position on startup
- **Smooth movement**: Adjustable sensitivity, deadzone, and smoothing
- **Plug-and-play**: Auto-detects Arduino serial port

## Hardware Requirements

### Components
- Arduino (Uno, Nano, or compatible)
- MPU6050 accelerometer/gyroscope module
- 2x push buttons
- USB cable for Arduino connection

### Wiring Diagram

```
MPU6050 -> Arduino
------------------
VCC     -> 5V or 3.3V
GND     -> GND
SDA     -> A4 (Uno/Nano) or SDA pin
SCL     -> A5 (Uno/Nano) or SCL pin

Buttons -> Arduino
------------------
Button 1 -> Pin 2 (other side to GND)
Button 2 -> Pin 3 (other side to GND)

Note: Internal pullup resistors are enabled in software,
      so no external resistors are needed for buttons.
```

## Software Requirements

### Arduino Libraries
Install these via Arduino Library Manager:
- `Adafruit MPU6050`
- `Adafruit Unified Sensor`

### Python Dependencies
```bash
pip install pyserial pynput pyautogui
```

## Installation

### 1. Flash Arduino Firmware

1. Open `revised/revised.ino` in Arduino IDE
2. Install required libraries (see Software Requirements)
3. Select your Arduino board and port
4. Click Upload

### 2. Run Python Controller

```bash
python mc_v2.py
```

The program will:
1. Auto-detect the Arduino
2. Calibrate for 500 samples (~2-3 seconds)
3. Activate mouse control

**Important**: Keep the controller stationary and level during calibration!

## How It Works

### System Architecture

```
┌─────────────┐    Serial USB     ┌──────────────┐
│   Arduino   │ ─────────────────>│   Computer   │
│             │   230400 baud     │              │
│  MPU6050    │   CSV data        │  Python      │
│  Buttons    │                   │  pynput      │
└─────────────┘                   └──────────────┘
```

### Data Flow

#### Arduino (revised.ino)
```
1. Read MPU6050 acceleration (X, Y, Z in m/s²)
2. Read button states (digital pins 2 and 3)
3. Transmit CSV line: "accelX,accelY,accelZ,btn1,btn2\n"
4. Repeat continuously (no delay for minimal latency)
```

#### Python (mc_v2.py)
```
1. Auto-detect Arduino serial port
2. Calibration phase:
   - Collect 500 acceleration samples
   - Calculate average (neutral position)
   - Store as calibration offsets

3. Mouse control loop:
   a. Parse CSV data from Arduino
   b. Handle button state changes (press/release)
   c. Calculate tilt relative to calibration
   d. Apply deadzone filtering
   e. Scale tilt to pixel offset
   f. Clamp to maximum offset (circular boundary)
   g. Apply exponential smoothing
   h. Convert to absolute screen position
   i. Move cursor (drag if button pressed)
```

### Control Algorithm

#### Coordinate Mapping
```
Sensor at rest     -> Cursor at screen center
Tilt forward       -> Cursor moves up from center
Tilt back          -> Cursor moves down from center
Tilt left          -> Cursor moves left from center
Tilt right         -> Cursor moves right from center
Return to neutral  -> Cursor returns to center
```

#### Deadzone Filtering
Small tilts below the deadzone threshold are ignored to prevent cursor drift:
```python
if abs(tilt) < DEADZONE:
    tilt = 0
else:
    # Remove deadzone to avoid discontinuity
    tilt = sign(tilt) * (abs(tilt) - DEADZONE)
```

#### Exponential Smoothing
Reduces jitter while maintaining responsiveness:
```python
smooth = α * old_smooth + (1-α) * new_target
```
Where α = SMOOTHING (0.7 by default)

#### Circular Boundary
Cursor is clamped to a circular region around screen center:
```python
distance = sqrt(x² + y²)
if distance > MAX_OFFSET:
    scale = MAX_OFFSET / distance
    x *= scale
    y *= scale
```

## Configuration

Edit these parameters at the top of `mc_v2.py`:

```python
# Movement parameters
SENSITIVITY_X = 300    # Pixels from center per 1G of tilt (horizontal)
SENSITIVITY_Y = 300    # Pixels from center per 1G of tilt (vertical)
DEADZONE = 0.2         # Ignore tilts smaller than this (in G's)
MAX_OFFSET = 600       # Maximum cursor distance from screen center (pixels)
SMOOTHING = 0.7        # Exponential smoothing (0=none, 1=maximum)
```

### Tuning Guide

**Cursor too sensitive/fast:**
- Decrease `SENSITIVITY_X` and `SENSITIVITY_Y` (try 200)

**Cursor not sensitive enough:**
- Increase `SENSITIVITY_X` and `SENSITIVITY_Y` (try 400)

**Cursor drifts when hand is at rest:**
- Increase `DEADZONE` (try 0.25 or 0.3)
- Re-calibrate by restarting the program

**Cursor is jittery/shaky:**
- Increase `SMOOTHING` (try 0.8 or 0.9)
- Note: Higher values add more lag

**Cursor feels laggy/sluggish:**
- Decrease `SMOOTHING` (try 0.5 or 0.6)
- Note: Lower values increase jitter

**Cursor can't reach screen edges:**
- Increase `MAX_OFFSET` (try 800 or 1000)

## Usage

### Basic Operation
1. Run `python mc_v2.py`
2. Keep controller level during calibration
3. Tilt controller to move cursor from center
4. Press buttons to click/drag

### Button Functions
- **Button 1 (Pin 2)**: Left mouse button
  - Click: Press and release quickly
  - Drag: Press, move cursor, then release

- **Button 2 (Pin 3)**: Right mouse button
  - Click: Press and release quickly
  - Drag: Press, move cursor, then release

### Tips
- **Smooth movements**: Move slowly and steadily
- **Precision work**: Use lower sensitivity settings
- **Gaming**: Use higher sensitivity, lower smoothing
- **Recalibrate**: Restart program if cursor drifts

## Troubleshooting

### Arduino not detected
```
Error: Arduino not found!
```
**Solutions:**
- Check USB cable connection
- Verify Arduino drivers are installed
- Try a different USB port
- Check Arduino IDE can see the port

### Cursor drifts constantly
**Cause**: Controller wasn't level during calibration

**Solution**: Restart program with controller on flat surface

### Right click button doesn't work
**Solutions:**
- Check button wiring to pin 3
- Test with multimeter (should show LOW when pressed)
- Swap button pins in Arduino code to test

### Cursor movement is reversed
**Solution**: Edit `mc_v2.py` line 215:
```python
# Change this:
tilt_x = -(x_val - calibration_offset_x)

# To this (remove the minus):
tilt_x = (x_val - calibration_offset_x)

# Or flip Y axis:
tilt_y = -(y_val - calibration_offset_y)
```

### Serial communication errors
```
Error: Read error: ...
```
**Solutions:**
- Check baud rate matches (230400) in both files
- Try unplugging and reconnecting Arduino
- Close Arduino Serial Monitor if open
- Restart the Python program

### MPU6050 not found
```
Error: Failed to find MPU6050 chip
```
**Solutions:**
- Check I2C wiring (SDA, SCL)
- Verify MPU6050 has power (3.3V or 5V)
- Try I2C scanner sketch to detect address
- Check for loose connections

## Technical Details

### Serial Protocol
- **Baud rate**: 230400 (high speed for low latency)
- **Format**: CSV (Comma-Separated Values)
- **Line ending**: `\n` (newline)
- **Data order**: `accelX,accelY,accelZ,btn1,btn2`
- **Update rate**: As fast as possible (~500-1000 Hz)

### Acceleration Values
- **Units**: m/s² (meters per second squared)
- **Typical range**: -10 to +10 m/s²
- **Gravity**: ~9.81 m/s² (always present on one axis)
- **1G of tilt**: ~9.81 m/s² change in that direction

### Calibration
- **Sample count**: 500 readings
- **Duration**: 2-3 seconds
- **Purpose**: Learn neutral position offset
- **Stored**: X, Y, Z calibration offsets
- **Applied**: Subtracted from all future readings

### Coordinate System
```
Arduino MPU6050:          Python Screen:
    Y (forward)              Y (down)
    ↑                        ↓
    |                        |
    +----> X (right)         +----> X (right)
   /                        /
  Z (up)                   Z (into screen)
```

Note: X axis is flipped in software for intuitive control

## File Structure

```
controller-project/
├── mc_v2.py                # Python mouse controller (main program)
├── revised/
│   └── revised.ino         # Arduino firmware
├── README.md               # This file
└── sketch/                 # Legacy/alternative sketches
    └── sketch.ino
```

## Credits

- **MPU6050**: Adafruit MPU6050 library
- **Mouse control**: pynput library
- **Serial communication**: pyserial library

## License

This project is open source and available for personal and educational use.

## Future Improvements

Possible enhancements:
- [ ] Gyroscope integration for rotation sensing
- [ ] Middle mouse button support
- [ ] Scroll wheel emulation (twist gesture)
- [ ] Multiple sensitivity profiles
- [ ] GUI for parameter adjustment
- [ ] Bluetooth wireless connection
- [ ] Gesture recognition (shake to recalibrate)
- [ ] Battery-powered portable version
