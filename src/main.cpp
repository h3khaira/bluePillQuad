#include<Arduino.h>
#include<Wire.h>

int16_t gyroRoll, gyroPitch, gyroYaw; //using int16 as mpu6050 returns a 16 bit two's complement value. using int16 will auto resolve it into a 16 bit number instead of 32 bit if you use int
int16_t gyroAddress = 0x68;
int16_t temperature; 
int16_t xAcc, yAcc, zAcc;

TwoWire Wire1(PB11, PB10); //setting pb11 and pb10 and I2C data and clock signals

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
  Wire1.write(0x06); //Digial Low Pass Filter for gyro/accelorometer set to 43 Hz
  Wire1.endTransmission();
  digitalWrite(PB4, HIGH);
}

void readOrientation(){
  Wire1.beginTransmission(gyroAddress);
  Wire1.write(0x43); //first register of accelerometer
  Wire1.endTransmission();
  Wire1.requestFrom(gyroAddress,14); //read 14 bytes of information starting from 0x3B register
  while (Wire1.available() < 14); //waiting for information to be delivered
  xAcc = Wire1.read() << 8 | Wire1.read(); // shift high byte left and add to low byte
  yAcc = Wire1.read() << 8 | Wire1.read();
  zAcc = Wire1.read() << 8 | Wire1.read();
  temperature = Wire1.read() << 8 | Wire1.read();
  // temperature = temperature/340 + 36.53;
  gyroRoll = Wire1.read() << 8 | Wire1.read();
  gyroPitch = Wire1.read() << 8 | Wire1.read();
  gyroYaw = Wire1.read() << 8 | Wire1.read();
}

void setup() {
  // delay(4000);
  Serial.begin(57600);
  Wire1.begin();
  delay(250);
  startGyro();
}

void loop() {
  readOrientation();
  Wire1.read() << 8 | Wire1.read();
  Serial.print("X = ");
  Serial.print(xAcc);
  Serial.print(" Y = ");
  Serial.print(yAcc);
  Serial.print(" Z = ");
  Serial.println(zAcc);
  delay(250);
}
