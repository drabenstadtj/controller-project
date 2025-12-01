// 3D Motion Controller with MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Timing
unsigned long lastTime = 0;

void setup(void) {
  Serial.begin(115200);
  
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
    
  // Update velocity (integrate acceleration)
  // velX += accelX * dt;
  // velY += accelY * dt;
  // velZ += accelZ * dt;
  
  // If velocity is very small, zero it out completely
  // if (abs(velX) < 0.05) velX = 0;
  // if (abs(velY) < 0.05) velY = 0;
  // if (abs(velZ) < 0.05) velZ = 0;
    
  // Output tracking data
  Serial.print(accelX, 3);
  Serial.print(",");
  Serial.print(accelY, 3);
  Serial.print(",");
  Serial.println(accelZ, 3);
  delay(1);  
}

// void printTrackingData(float accelX, float accelY, float accelZ) {
//   Serial.print("ACC:");
//   Serial.print(velX, 3);
//   Serial.print(",");
//   Serial.print(velY, 3);
//   Serial.print(",");
//   Serial.println(velZ, 3);
// }