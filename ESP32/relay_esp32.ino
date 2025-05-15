#include "MPU9250.h"
#include <Wire.h> // Include this if using I2C devices like IMU

const int relayPin1 = 13; // GPIO pin for side brush 1 and 2 
const int relayPin3 = 27; // GPIO pin for main brush 

// IMU sensor setup
//MPU9250 IMU(Wire, 0x68);
//int status;

// Manually calibrated offsets
//float gyroX_offset = 0.000022;
//float gyroY_offset = 0.000290;
//float gyroZ_offset = -0.000068;
//float accelX_offset = -0.242535;
//float accelY_offset = 1.668020;
//float accelZ_offset = -21.458786;

// Threshold and filtering 
//const float threshold = 0.01;

void setup(){
  Serial.begin(115200);
  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin3, OUTPUT);
  digitalWrite(relayPin1, LOW);
  digitalWrite(relayPin3, LOW);

  // Initialize IMU 
  //IMUSetup();
}


void loop(){
  handleRelayControl(); 
  //handleIMUData();
  //delay(300);  // Optional: Small delay to avoid rapid polling
}

//void IMUSetup(){
  //status = IMU.begin();
  //if(status < 0){
    //Serial.println("IMU Initialization unsuccessful");
    //delay(500);
    //while(1){}
 // }
//}

void handleRelayControl(){
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "RELAY1_ON") {
      digitalWrite(relayPin1, HIGH);
    } else if (command == "RELAY1_OFF") {
      digitalWrite(relayPin1, LOW);
    }

    if (command == "RELAY3_ON") {
      digitalWrite(relayPin3, HIGH);
    } else if (command == "RELAY3_OFF") {
      digitalWrite(relayPin3, LOW);
    }
  }
}

//void handleIMUData(){
  //IMU.readSensor();
  //float gyroX = applyThreshold(IMU.getGyroX_rads() - gyroX_offset);
  //float gyroY = applyThreshold(IMU.getGyroY_rads() - gyroY_offset);
  //float gyroZ = applyThreshold(IMU.getGyroZ_rads() - gyroZ_offset);

  //float accelX = applyThreshold(IMU.getAccelX_mss() - accelX_offset);
  //float accelY = applyThreshold(IMU.getAccelY_mss() - accelY_offset);
  //float accelZ = applyThreshold(IMU.getAccelZ_mss() - accelZ_offset);

  //Serial.print("IMU Data:\n");
  //Serial.print("Angular velocity:\n");
  //Serial.print("    x: "); Serial.print(gyroX, 2); Serial.print("\n");
  //Serial.print("    y: "); Serial.print(gyroY, 2); Serial.print("\n");
  //Serial.print("    z: "); Serial.print(gyroZ, 2); Serial.print("\n");
  //Serial.print("  Linear acceleration:\n");
  //Serial.print("    x: "); Serial.print(accelX, 2); Serial.print("\n");
  //Serial.print("    y: "); Serial.print(accelY, 2); Serial.print("\n");
  //Serial.print("    z: "); Serial.print(accelZ, 2); Serial.print("\n");
//}

//float applyThreshold(float value){
  //return (abs(value) < threshold) ? 0.0 : value;
//}
