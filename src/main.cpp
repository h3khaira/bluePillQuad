#include<Arduino.h>
#include<Wire.h>

int gyro_x, gyro_y, gyro_z;
 
void setup() {
  pinMode(PB3, OUTPUT);
  Serial.begin(57600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); //power management register
  Wire.write(0x00); // waking up gyro
  digitalWrite(PB4, HIGH);
  Wire.endTransmission();
}
 
void loop() {
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  gyro_x = Wire.read()<<8 | Wire.read();
  gyro_y = Wire.read()<<8 | Wire.read();
  gyro_z = Wire.read()<<8 | Wire.read();
  Serial.print(gyro_x);
  delay(250);
}