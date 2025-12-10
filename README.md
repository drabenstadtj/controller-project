## File Breakdown

- **`mouse_controllerv1.py`** - Original version using 3D motion tracking
- **`mouse_controllerv2.py`** - Final version using joystick-style control  
- **`initial_sketch.ino`** - First Arduino sketch with on-board calculations
- **`sensor_button_serial.ino`** - Final Arduino firmware (raw sensor data only)
- **`caterpillar_quest_controller_build/`** - Game build for the controller

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
2. Install required libraries
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
4. Repeat continuously
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
Where α = SMOOTHING 

#### Circular Boundary
Cursor is clamped to a circular region around screen center:
```python
distance = sqrt(x² + y²)
if distance > MAX_OFFSET:
    scale = MAX_OFFSET / distance
    x *= scale
    y *= scale
```

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


