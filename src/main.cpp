#include<Arduino.h>
#include<Wire.h>

int gyro_x, gyro_y, gyro_z;
TwoWire Wire1(PB11, PB10);

void setup() {
  Serial.begin(57600);
  pinMode(PB3, OUTPUT);
  digitalWrite(PB3, HIGH);
  Wire1.begin();
  delay(250);
  
  Wire1.beginTransmission(0x68);
  Wire1.write(0x6B);
  Wire1.write(0x00);
  Wire1.endTransmission();
}

// the loop function runs over and over again forever
void loop() {
  Wire1.beginTransmission(0x68);
  Wire1.write(0x43);
  Wire1.endTransmission();
  Wire1.requestFrom(0x68,6);
  gyro_x = Wire1.read()<<8 | Wire1.read();
  gyro_y = Wire1.read()<<8 | Wire1.read();
  gyro_z = Wire1.read()<<8 | Wire1.read();
  Serial.print("X = ");
  Serial.print(gyro_x);
  Serial.print(" Y = ");
  Serial.print(gyro_y);
  Serial.print(" Z = ");
  Serial.println(gyro_z);
  delay(250);
}