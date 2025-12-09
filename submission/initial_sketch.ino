// 3D Motion Controller with MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Position tracking
float posX = 0, posY = 0, posZ = 0;
float velX = 0, velY = 0, velZ = 0;

// Calibration values
float offsetX = 0, offsetY = 0, offsetZ = 0;

// Timing
unsigned long lastTime = 0;

// States
bool isCalibrated = false;
bool isTracking = false;

// IMPROVED THRESHOLDS - More aggressive
const float MOVEMENT_THRESHOLD = 0.8;   // Increased from 0.5 - ignore small movements
const float VELOCITY_DECAY = 0.70;      // Increased from 0.95 - faster return to zero
const float VELOCITY_LIMIT = 2.0;       // Cap maximum velocity to prevent spikes

void setup(void) {
  Serial.begin(2000000);
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  // Configure sensor for responsive tracking
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);  // Lower bandwidth = smoother but less responsive
  
  Serial.println("=== 3D Motion Controller ===");
  Serial.println("Commands:");
  Serial.println("  'c' - Calibrate starting position");
  Serial.println("  's' - Start tracking");
  Serial.println("  'r' - Reset position");
  Serial.println();
  
  delay(100);
  lastTime = millis();
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    handleCommand(cmd);
  }
  
  // Get sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calculate time delta
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  if (isTracking && isCalibrated) {
    // Remove calibration offset and gravity
    float accelX = a.acceleration.x - offsetX;
    float accelY = a.acceleration.y - offsetY;
    float accelZ = a.acceleration.z - offsetZ;
    
    // Apply movement threshold (ignore sensor noise)
    if (abs(accelX) < MOVEMENT_THRESHOLD) accelX = 0;
    if (abs(accelY) < MOVEMENT_THRESHOLD) accelY = 0;
    if (abs(accelZ) < MOVEMENT_THRESHOLD) accelZ = 0;
    
    // Update velocity (integrate acceleration)
    velX += accelX * dt;
    velY += accelY * dt;
    velZ += accelZ * dt;
    
    // Apply velocity decay (prevents drift) - MORE AGGRESSIVE
    velX *= VELOCITY_DECAY;
    velY *= VELOCITY_DECAY;
    velZ *= VELOCITY_DECAY;
    
    // Limit maximum velocity to prevent spikes
    velX = constrain(velX, -VELOCITY_LIMIT, VELOCITY_LIMIT);
    velY = constrain(velY, -VELOCITY_LIMIT, VELOCITY_LIMIT);
    velZ = constrain(velZ, -VELOCITY_LIMIT, VELOCITY_LIMIT);
    
    // If velocity is very small, zero it out completely
    if (abs(velX) < 0.05) velX = 0;
    if (abs(velY) < 0.05) velY = 0;
    if (abs(velZ) < 0.05) velZ = 0;
    
    // Update position (integrate velocity)
    posX += velX * dt;
    posY += velY * dt;
    posZ += velZ * dt;
    
    // Output tracking data
    printTrackingData(accelX, accelY, accelZ);
  }
  
  delay(1);  
}

void handleCommand(char cmd) {
  switch(cmd) {
    case 'c':
    case 'C':
      calibrate();
      break;
      
    case 's':
    case 'S':
      if (!isCalibrated) {
        Serial.println("Please calibrate first ('c')");
      } else {
        isTracking = !isTracking;
        if (isTracking) {
          Serial.println("Started tracking!");
          resetPosition();
        } else {
          Serial.println("Stopped tracking");
        }
      }
      break;
      
    case 'r':
    case 'R':
      resetPosition();
      Serial.println("Position reset");
      break;
  }
}

void calibrate() {
  Serial.println("Calibrating... Hold still!");
  
  // Take multiple samples
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 100;  // Increased from 50 for better calibration
  
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumX += a.acceleration.x;
    sumY += a.acceleration.y;
    sumZ += a.acceleration.z;
    delay(10);
  }
  
  // Calculate average (resting position)
  offsetX = sumX / samples;
  offsetY = sumY / samples;
  offsetZ = sumZ / samples;
  
  isCalibrated = true;
  Serial.println("Calibration complete!");
  Serial.print("Offsets - X: ");
  Serial.print(offsetX);
  Serial.print(" Y: ");
  Serial.print(offsetY);
  Serial.print(" Z: ");
  Serial.println(offsetZ);
  Serial.println("Ready to track. Press 's' to start.");
}

void resetPosition() {
  posX = posY = posZ = 0;
  velX = velY = velZ = 0;
}

void printTrackingData(float accelX, float accelY, float accelZ) {
  // Only print if there's actual movement to reduce serial traffic
  // if (abs(velX) > 0.01 || abs(velY) > 0.01 || abs(velZ) > 0.01) {
  Serial.print("VEL:");
  Serial.print(velX, 3);
  Serial.print(",");
  Serial.print(velY, 3);
  Serial.print(",");
  Serial.println(velZ, 3);
  // }
}