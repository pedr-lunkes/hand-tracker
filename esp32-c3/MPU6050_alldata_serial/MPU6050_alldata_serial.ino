#include <MPU6050_WE.h>
#include <Wire.h>
#define MPU6050_ADDR 0x68

MPU6050_WE myMPU6050 = MPU6050_WE(MPU6050_ADDR);

void setup() {
  Serial.begin(115200);
  Wire.begin(6, 7);
  if(!myMPU6050.init()){
    Serial.println("MPU6050 does not respond");
  }
  else{
    Serial.println("MPU6050 is connected");
  }
  
  Serial.println("Position you MPU6050 flat and don't move it - calibrating...");
  delay(1000);
  myMPU6050.autoOffsets();
  Serial.println("Done!");

  myMPU6050.setGyrRange(MPU6050_GYRO_RANGE_250);
  myMPU6050.setAccRange(MPU6050_ACC_RANGE_2G);
  
  delay(200);
}

void loop() {
  xyzFloat gValue = myMPU6050.getGValues();
  xyzFloat gyr = myMPU6050.getGyrValues();

  // Acceleration
  Serial.print(gValue.x);
  Serial.print(" ");
  Serial.print(gValue.y);
  Serial.print(" ");
  Serial.print(gValue.z);
  Serial.print(" ");

  // Gyroscope
  Serial.print(gyr.x);
  Serial.print(" ");
  Serial.print(gyr.y);
  Serial.print(" ");
  Serial.print(gyr.z);
  Serial.print("\n");

  delay(10);
}
