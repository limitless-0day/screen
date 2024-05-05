#include "Wire.h"

void setup() {
  Serial.begin(115200);
  Wire.begin(8,9);e:\Project\ESP32-S3-Touch-LCD-7\Code\ESP32-S3-Touch-LCD-7_Code\Arduino-ESP32-3.0.0\Sensor_AD\Sensor_AD.ino
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning for I2C devices ...");
  for(address = 0x01; address < 0x7f; address++){
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0){
      Serial.printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    } else if(error != 2){
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0){
    Serial.println("No I2C devices found");
  }
  delay(5000);
}
