#include <Arduino.h>
#include <Wire.h>
#include "AttitudeEstimator.h"

#define MPU_ADDR 0x68 // I2C address of the MPU-6050

unsigned long prevTime = millis();

AttitudeEstimator attitudeEstimator;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Wake up the MPU-6050 (initially in sleep mode)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power management register
  Wire.write(0);    // Set to 0 = wake up MPU-6050
  Wire.endTransmission(true);
}

void loop() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Request 14 bytes

  int16_t ax = Wire.read() << 8 | Wire.read(); // Accel X
  int16_t ay = Wire.read() << 8 | Wire.read(); // Accel Y
  int16_t az = Wire.read() << 8 | Wire.read(); // Accel Z
  int16_t gx = Wire.read() << 8 | Wire.read(); // Gyro X
  int16_t gy = Wire.read() << 8 | Wire.read(); // Gyro Y
  int16_t gz = Wire.read() << 8 | Wire.read(); // Gyro Z

  const BLA::Matrix<3, 1> gyro(gx, gy, gz);
  const BLA::Matrix<3, 1> accel(ax, ay, az);

  attitudeEstimator.update(gyro, accel);

  unsigned long currentTime = millis();
  if (currentTime - prevTime >= 1000) 
  { // Print every second
    Serial.print("Accel: ");
    Serial.print(ax); Serial.print(", ");
    Serial.print(ay); Serial.print(", ");
    Serial.print(az); Serial.print(" | ");

    Serial.print("Gyro: ");
    Serial.print(gx); Serial.print(", ");
    Serial.print(gy); Serial.print(", ");
    Serial.print(gz); Serial.println();

    Serial.print("Orientation: ");
    BLA::Matrix<4, 1> orientation = attitudeEstimator.getOrientation();
    Serial.print(orientation(0)); Serial.print(", ");
    Serial.print(orientation(1)); Serial.print(", ");
    Serial.print(orientation(2)); Serial.println();

    prevTime = currentTime;
  }
}