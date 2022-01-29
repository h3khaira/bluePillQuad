#include<Arduino.h>
#include<Wire.h>

int gyro_x, gyro_y, gyro_z;
 
void setup() {
  delay(2000);
  pinMode(PB3, OUTPUT);
  digitalWrite(PB3, HIGH);
  SerialUSB.begin(115200);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); //power management register
  Wire.write(0x00); // waking up gyro
  Wire.endTransmission();
}
 
void loop() {
  SerialUSB.println("helll0o");
  delay(1000);
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  gyro_x = Wire.read()<<8 | Wire.read();
  gyro_y = Wire.read()<<8 | Wire.read();
  gyro_z = Wire.read()<<8 | Wire.read();
  SerialUSB.println("hello9");
  delay(250);
}