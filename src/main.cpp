#include<Arduino.h>
#include<Wire.h>

int16_t gyro_x, gyro_y, gyro_z; //using int16 as mpu6050 returns a 16 bit two's complement value. using int16 will auto resolve it into a 16 bit number instead of 32 bit if you use int
TwoWire Wire1(PB11, PB10);

void setup() {
  Serial.begin(57600);
  pinMode(PB3, OUTPUT);
  digitalWrite(PB3, HIGH);
  Wire1.setClock(400000);
  Wire1.begin();
  delay(250);
  
  Wire1.beginTransmission(0x68); //MPU-6050 address
  Wire1.write(0x6B); //Access power management register
  Wire1.write(0x00); //wake up the gyro
  Wire1.endTransmission();
}

// the loop function runs over and over again forever
void loop() {
  Wire1.beginTransmission(0x68); //mpu-6050 address
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