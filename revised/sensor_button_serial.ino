/*
 * 3D Motion Controller - Arduino Firmware
 * ========================================
 * Reads data from an MPU6050 accelerometer and two push buttons,
 * then transmits the data to a PC via serial communication.
 *
 * Hardware Setup:
 * - MPU6050 connected via I2C (SDA, SCL)
 * - Button 1 on pin 2 (internal pullup resistor enabled)
 * - Button 2 on pin 3 (internal pullup resistor enabled)
 *
 * Data Format (CSV over serial at 230400 baud):
 *   accelX,accelY,accelZ,button1,button2
 *
 * Example output:
 *   0.123,-0.456,9.810,0,0    (no buttons pressed)
 *   0.234,-0.567,9.820,1,0    (button 1 pressed)
 *   0.345,-0.678,9.830,0,1    (button 2 pressed)
 */

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

Adafruit_MPU6050 mpu;

// Button pins (connect buttons between pin and GND)
const int BUTTON1_PIN = 2;  // Left mouse button
const int BUTTON2_PIN = 3;  // Right mouse button

// Timing variables (currently unused but available for future use)
unsigned long lastTime = 0;

// ============================================================================
// INITIALIZATION
// ============================================================================

void setup(void) {
  // Initialize serial communication at high baud rate for low latency
  Serial.begin(230400);

  // Configure button pins with internal pullup resistors
  // This means: HIGH = not pressed, LOW = pressed
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);

  // Initialize MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    // Halt execution if sensor not found
    while (1) {
      delay(10);
    }
  }

  // Configure sensor ranges and filtering
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);      // ±8G range for tilt sensing
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);           // ±500°/s (not currently used)
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);       // 260Hz bandwidth for responsive tracking
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // ------------------------------------------------------------------------
  // 1. READ SENSOR DATA
  // ------------------------------------------------------------------------
  // Get latest accelerometer, gyroscope, and temperature readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate time delta since last reading (for future use)
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // Convert to seconds
  lastTime = currentTime;

  // Extract acceleration values (in m/s²)
  // These values represent both gravity and tilt of the sensor
  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;

  // ------------------------------------------------------------------------
  // 2. READ BUTTON STATES
  // ------------------------------------------------------------------------
  // Read digital pins (inverted because INPUT_PULLUP means LOW = pressed)
  bool btn1 = !digitalRead(BUTTON1_PIN);
  bool btn2 = !digitalRead(BUTTON2_PIN);

  // ------------------------------------------------------------------------
  // 3. TRANSMIT DATA
  // ------------------------------------------------------------------------
  // Send data in CSV format: accelX,accelY,accelZ,btn1,btn2
  // Acceleration printed with 3 decimal places for precision
  Serial.print(accelX, 3);
  Serial.print(",");
  Serial.print(accelY, 3);
  Serial.print(",");
  Serial.print(accelZ, 3);
  Serial.print(",");
  Serial.print(btn1 ? "1" : "0");
  Serial.print(",");
  Serial.println(btn2 ? "1" : "0");  // println adds newline delimiter

  // No delay - loop runs as fast as possible for minimal latency
}
