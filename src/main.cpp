#include<Arduino.h>
#include<Wire.h>

int16_t gyro_x, gyro_y, gyro_z; //using int16 as mpu6050 returns a 16 bit two's complement value. using int16 will auto resolve it into a 16 bit number instead of 32 bit if you use int
TwoWire Wire1(PB11, PB10);
int gyroAddress = 0x68;

void startGyro(){
  Wire1.beginTransmission(gyroAddress);
  Wire1.write(0x6B); //power management register
  Wire1.write(0x00); //setting this bit wakes up the MPU-6050 module
  Wire1.endTransmission();

  Wire1.beginTransmission(gyroAddress);
  Wire1.write(0x1B); //Access the GYRO_CONFIG register
  Wire1.write(0x08); //Write the FS_SEL bits to change gyro resolution to 500 degrees per second, 1 DEG/SEC  = 65.5 OUTPUT FROM GYRO LSB
  Wire1.endTransmission();

  Wire1.beginTransmission(gyroAddress);
  Wire1.write(0x1C); //Access accelerometer config register
  Wire1.write(0x10); // +-8g = 4096 LSB/g
  Wire1.endTransmission();

  Wire1.beginTransmission(gyroAddress);
  Wire1.write(0x1A); //Access CONFIG register
  Wire1.write(0x06); // Digial Low Pass Filter for gyro/accelorometer set to 43 Hz
  Wire1.endTransmission();
}

void readOrientation(){
  Wire1.beginTransmission(gyroAddress);
  Wire1.write(0x3B); //first register of accelerometer
  Wire1.endTransmission();
  Wire1.requestFrom(gyroAddress,14); //read 14 bytes of information starting from 0x3B registerWire1
  while (Wire1.available() < 14); //waiting for information to be delivered
}

void setup() {
  // delay(4000);
  Serial.begin(57600);
  pinMode(PB3, OUTPUT);
  digitalWrite(PB3, HIGH);
  Wire1.begin();
  delay(250);
  Wire1.setClock(400000);
  Serial.println("Talking to gyro"); 
  startGyro();
}

void loop() {
  readOrientation();
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
