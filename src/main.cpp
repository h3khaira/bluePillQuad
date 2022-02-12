#include <Arduino.h>
#include <Wire.h>

// register addresses for receiver input capture mode
#define TIM2_CR1 (*(__IO uint32_t *)0x40000000)   // Control register 1, the timer has two registers dedicated to it. Page 406
#define TIM2_CR2 (*(__IO uint32_t *)0x40000004)   // Control register 2
#define TIM2_SMCR (*(__IO uint32_t *)0x40000008)  // TIM2 slave mode control register
#define TIM2_DIER (*(__IO uint32_t *)0x4000000C)  // Interrupt enable register
#define TIM2_EGR (*(__IO uint32_t *)0x40000010)   // event generation register
#define TIM2_CCMR1 (*(__IO uint32_t *)0x40000018) // capture compare mode register 1. This register is used to link the input from the receiver to the interrupt.
#define TIM2_CCMR2 (*(__IO uint32_t *)0x4000001C) // capture compare mode register 2
#define TIM2_CCER (*(__IO uint32_t *)0x40000020)
#define TIM2_PSC (*(__IO uint32_t *)0x40000028)
#define TIM2_ARR (*(__IO uint32_t *)0x4000002C)
#define TIM2_DCR (*(__IO uint32_t *)0x40000048)

// using int16 as mpu6050 returns a 16 bit two's complement value. using int16 will auto resolve it into a 16 bit number instead of 32 bit if you use int
int16_t rawGyroRoll, rawGyroPitch, rawGyroYaw;       // gyroscope measurement, resolution based on value of 1B register
float rollCal = -298, pitchCal = -175, yawCal = 230; // for calibration ofgyro measurements
int16_t gyroAddress = 0x68;                          // MPU-6050 I2C address
int16_t rawTemp;
float temperature;
int16_t rawXAcc, rawYAcc, rawZAcc; // Raw gyro acceleration values 4096 least significant bits per 9.81 m/s^2
float accRoll, accPitch;           // to store actual acceleration numbers
float roll, pitch, yaw;

float mpuFilterWeight = 0.96;
uint32_t loopTimer;

TwoWire Wire1(PB11, PB10); // setting pb11 and pb10 and I2C data and clock signals

void startGyro()
{
  Wire1.beginTransmission(gyroAddress);
  Wire1.write(0x6B); // power management register
  Wire1.write(0x00); // setting this bit wakes up the MPU-6050 module
  Wire1.endTransmission();

  Wire1.beginTransmission(gyroAddress);
  Wire1.write(0x1B); // Access the GYRO_CONFIG register
  Wire1.write(0x08); // Write the FS_SEL bits to change gyro resolution to 500 degrees per second, 1 DEG/SEC  = 65.5 OUTPUT FROM GYRO LSB (check page 31 of manual)
  Wire1.endTransmission();

  Wire1.beginTransmission(gyroAddress);
  Wire1.write(0x1C); // Access accelerometer config register
  Wire1.write(0x10); // Set the AFSL bit +-8g = 4096 LSB/g
  Wire1.endTransmission();

  Wire1.beginTransmission(gyroAddress);
  Wire1.write(0x1A); // Access CONFIG register
  Wire1.write(0x03); // Digial Low Pass Filter for gyro/accelorometer set to 43 Hz
  Wire1.endTransmission();
}

void eulerAngles()
{
  accRoll = 57.2958 * atan2(rawYAcc, rawZAcc);
  accPitch = 57.2958 * atan2(rawXAcc, sqrt(pow(rawYAcc, 2) + pow(rawZAcc, 2)));
}

void readOrientation()
{
  Wire1.beginTransmission(gyroAddress);
  Wire1.write(0x3B); // first register of accelerometer
  Wire1.endTransmission();
  Wire1.requestFrom(gyroAddress, 14); // read 14 bytes of information starting from 0x3B register
  while (Wire1.available() < 14)
    ;                                         // waiting for information to be delivered
  rawXAcc = Wire1.read() << 8 | Wire1.read(); // shift high byte left and add to low byte
  rawYAcc = Wire1.read() << 8 | Wire1.read();
  rawZAcc = Wire1.read() << 8 | Wire1.read();
  rawTemp = Wire1.read() << 8 | Wire1.read();
  // temperature = rawTemp/340.0 + 36.53; //uncomment this to read temperature
  rawGyroRoll = Wire1.read() << 8 | Wire1.read();
  rawGyroPitch = Wire1.read() << 8 | Wire1.read();
  rawGyroYaw = Wire1.read() << 8 | Wire1.read();

  // calibrating raw gyro values based on offsets measured in setup
  rawGyroPitch -= pitchCal;
  rawGyroRoll -= rollCal;
  rawGyroYaw -= yawCal;

  eulerAngles();

  // 0.000061 = 1/250/65.5
  roll += rawGyroRoll * 0.0000611; // integrating gyroscope velocity measurements by using sampling rate of 250 Hz
  pitch += rawGyroPitch * 0.0000611;

  roll = mpuFilterWeight * roll + (1 - mpuFilterWeight) * accRoll;    // combining accelerometer and gyroscope measurements
  pitch = mpuFilterWeight * pitch + (1 - mpuFilterWeight) * accPitch; // combining accelerometer and gyroscope measurements
}

void setup()
{
  Serial.begin(57600);
  // Setting input capture registers
  TIM2_CR1 = 2; // setting the CEN bit to 1, this enables the counter
  TIM2_CR2 = 0;
  TIM2_SMCR = 0;
  TIM2_DIER = 2; // set the CC1IE bit to enable CC1 interrupt
  TIM2_EGR = 0;
  TIM2_CCMR1 = 2;
  TIM2_CCMR2 = 0;
  TIM2_CCER = 1;     // set the CC1E bit to 1, enabling capture
  TIM2_PSC = 72;     // set prescaler value to 72 so timer updates at 1 Mhz
  TIM2_ARR = 0xFFFF; // set value of the autoload register to 65535, this is the highest value the counter can count to

  NVIC_EnableIRQ(TIM2_IRQn); // Enable timer 2 interrupt
  pinMode(PB3, OUTPUT);
  pinMode(PB4, OUTPUT);
  digitalWrite(PB4, LOW);
  Wire1.begin();
  delay(250);
  startGyro();
  loopTimer = micros();
}

void loop()
{
  readOrientation();
  // Serial.print("Roll = ");
  // Serial.print(roll);
  // Serial.print(" Pitch = ");
  // Serial.println(pitch);

  if (micros() - loopTimer > 4050)
    digitalWrite(PB4, HIGH); // throw an error if refresh rate is lower than 250 Hz, this affects angle calculation
  while (micros() - loopTimer < 4000)
    ; // We wait until 4000us are passed.
  loopTimer = micros();
}

extern "C" void TIM2_IRQHandler(void)
{
  digitalWrite(PB3, HIGH);
  Serial.println("Interrup triggered");
  TIM2->SR &= ~TIM_SR_CC1IF;
}
