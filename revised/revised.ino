// 3D Motion Controller with MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Timing
unsigned long lastTime = 0;

// BUTTON CONFIGURATION
const int BUTTON1_PIN = 2;  // Change to your actual pins
const int BUTTON2_PIN = 3;

void setup(void) {
  Serial.begin(230400);
  
  // Setup button pins with internal pullups
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  // Configure sensor for responsive tracking
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ); 
}

void loop() {  
  // Get sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calculate time delta
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // Remove calibration offset and gravity
  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;
  
  // Read buttons (inverted because of pullup - LOW = pressed)
  bool btn1 = !digitalRead(BUTTON1_PIN);
  bool btn2 = !digitalRead(BUTTON2_PIN);
  
  // Output tracking data with button flags
  Serial.print(accelX, 3);
  Serial.print(",");
  Serial.print(accelY, 3);
  Serial.print(",");
  Serial.print(accelZ, 3);
  Serial.print(",");
  Serial.print(btn1 ? "1" : "0");
  Serial.print(",");
  Serial.println(btn2 ? "1" : "0");
} 