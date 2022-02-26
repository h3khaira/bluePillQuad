#include <Arduino.h>
#include <Wire.h>

#define pinGroupTimer1 PA2
#define pinGroupTimer2 PA6
#define pinGroupTimer3 PB6

volatile int32_t rawReceiverLeftRoller, timer2Channel1Rise;
volatile int32_t rawReceiverRightRoller, timer2Channel2Rise;
volatile int32_t rawReceiverYaw, timer2Channel3Rise;
volatile int32_t rawReceiverThrottle, timer2Channel4Rise;
volatile int32_t rawReceiverPitch, timer3Channel1Rise;
volatile int32_t rawReceiverRoll, timer3Channel2Rise;

HardwareTimer *timer2;
HardwareTimer *timer3;
HardwareTimer *timer4;

// using int16 as mpu6050 returns a 16 bit two's complement value. using int16 will auto resolve it into a 16 bit number instead of 32 bit if you use int
int16_t rawGyroRoll, rawGyroPitch, rawGyroYaw;       // gyroscope measurement, resolution based on value of 1B register
float rollCal = -298, pitchCal = -175, yawCal = 230; // for calibration ofgyro measurements
int16_t gyroAddress = 0x68;                          // MPU-6050 I2C address
int16_t altimeterAddress = 0x77;
int16_t rawTemp;
float temperature;
int16_t rawXAcc, rawYAcc, rawZAcc; // Raw gyro acceleration values 4096 least significant bits per 9.81 m/s^2
float accRoll, accPitch;           // to store actual acceleration numbers
float roll, pitch, yaw;

float mpuFilterWeight = 0.96;
uint32_t loopTimer;

uint16_t c[6];

TwoWire Wire1(PB11, PB10); // setting pb11 and pb10 and I2C data and clock signals

void handlerLeftRoller(void) // left roller on transmitter
{
  if (0b1 & GPIOA->IDR >> 0) // check if the A0 pin is high
  {
    timer2Channel1Rise = TIM2->CCR1;
    TIM2->CCER |= TIM_CCER_CC1P;
  }
  else
  {
    rawReceiverLeftRoller = TIM2->CCR1 - timer2Channel1Rise;
    if (rawReceiverLeftRoller < 0)
      rawReceiverLeftRoller += 0xFFFF;
    TIM2->CCER &= ~TIM_CCER_CC1P;
  }
}
void handlerRightRoller(void) // right roller on transmitter
{
  if (0b1 & GPIOA->IDR >> 1) // check if the A1 pin is high
  {
    timer2Channel2Rise = TIM2->CCR2;
    TIM2->CCER |= TIM_CCER_CC2P;
  }
  else
  {
    rawReceiverRightRoller = TIM2->CCR2 - timer2Channel2Rise;
    if (rawReceiverRightRoller < 0)
      rawReceiverRightRoller += 0xFFFF;
    TIM2->CCER &= ~TIM_CCER_CC2P;
  }
}
void handlerYawReceiver(void) // yaw control
{
  if (0b1 & GPIOA->IDR >> 2)
  { // If the input coming in from the IDR registers in high
    timer2Channel3Rise = TIM2->CCR3;
    TIM2->CCER |= TIM_CCER_CC3P;
  }
  else
  {
    rawReceiverYaw = TIM2->CCR3 - timer2Channel3Rise;
    if (rawReceiverYaw < 0)
      rawReceiverYaw += 0xFFFF;
    TIM2->CCER &= ~TIM_CCER_CC3P;
  }
}

void handlerThrottleReceiver(void) // throttle
{
  if (0b1 & GPIOA->IDR >> 3)
  { // If the input coming in from the IDR registers in high
    timer2Channel4Rise = TIM2->CCR4;
    TIM2->CCER |= TIM_CCER_CC4P;
  }
  else
  {
    rawReceiverThrottle = TIM2->CCR4 - timer2Channel4Rise;
    if (rawReceiverThrottle < 0)
      rawReceiverThrottle += 0xFFFF;
    TIM2->CCER &= ~TIM_CCER_CC4P;
  }
}

void handlerPitcReceiver(void) // pitch control
{
  if (0b1 & GPIOA->IDR >> 6)
  { // If the input coming in from the IDR registers in high
    timer3Channel1Rise = TIM3->CCR1;
    TIM3->CCER |= TIM_CCER_CC1P;
  }
  else
  {
    rawReceiverPitch = TIM3->CCR1 - timer3Channel1Rise;
    if (rawReceiverPitch < 0)
      rawReceiverPitch += 0xFFFF;
    TIM3->CCER &= ~TIM_CCER_CC1P;
  }
}

void handlerRollReceiver(void) // roll control
{
  if (0b1 & GPIOA->IDR >> 7)
  { // If the input coming in from the IDR registers in high
    timer3Channel2Rise = TIM3->CCR2;
    TIM3->CCER |= TIM_CCER_CC2P;
  }
  else
  {
    rawReceiverRoll = TIM3->CCR2 - timer3Channel2Rise;
    if (rawReceiverRoll < 0)
      rawReceiverRoll += 0xFFFF;
    TIM3->CCER &= ~TIM_CCER_CC2P;
  }
}

void setupTimers()
{
  // setting up timer 2 to serve first four channels on receiver
  TIM_TypeDef *instanceTimer2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pinGroupTimer1), PinMap_PWM);
  timer2 = new HardwareTimer(instanceTimer2);
  timer2->attachInterrupt((uint32_t)1, handlerLeftRoller);       // attach interrupt to timer 2 channel 1
  timer2->attachInterrupt((uint32_t)2, handlerRightRoller);      // attach interrupt to timer 2 channel 2
  timer2->attachInterrupt((uint32_t)3, handlerYawReceiver);      // attach interrupt to timer 2 channel 3
  timer2->attachInterrupt((uint32_t)4, handlerThrottleReceiver); // attach interrupt to timer 2 channel 4
  TIM2->CR1 = TIM_CR1_CEN;                                       // this enables the counter clock
  TIM2->CR2 = 0;
  TIM2->SMCR = 0;                                                                 // this ensures the internal clock is used by the timer
  TIM2->DIER = TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE; // capture/compare 1,2,3,4 interrupt enable
  TIM2->EGR = 0;
  TIM2->CCMR1 = 0b100000001;                                                  // set CC1S and CC2S bits to 01
  TIM2->CCMR2 = 0b100000001;                                                  // set CC3S and CC4S bits to 01
  TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; // configure the capture compare cannels to input and detect rising edges
  TIM2->PSC = 71;                                                             // set prescaler value to 72 so timer updates at 1 Mhz
  TIM2->ARR = 0xFFFF;                                                         // set value of the autoload register to 65535, this is the highest value the counter can count to
  TIM2->DCR = 0;

  // setting up timer 3 to serve the last two channels on receiver
  TIM_TypeDef *instanceTimer3 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pinGroupTimer2), PinMap_PWM);
  timer3 = new HardwareTimer(instanceTimer3);
  timer3->attachInterrupt((uint32_t)1, handlerPitcReceiver); // attach interrupt to timer 3 channel 1
  timer3->attachInterrupt((uint32_t)2, handlerRollReceiver); // attach interrupt to timer 3 channel 2
  TIM3->CR1 = TIM_CR1_CEN;                                   // this enables the counter clock
  TIM3->CR2 = 0;
  TIM3->SMCR = 0;                               // this ensures the internal clock is used by the timer
  TIM3->DIER = TIM_DIER_CC1IE | TIM_DIER_CC2IE; // capture/compare 1,2 interrupt enable
  TIM3->EGR = 0;
  TIM3->CCMR1 = 0b100000001; // set CC1S and CC2S bits to 01 to enable channels 1 and 2
  TIM3->CCMR2 = 0;
  TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E; // configure the capture compare cannels to input and detect rising edges
  TIM3->PSC = 71;                             // set prescaler value to 72 so timer updates at 1 Mhz
  TIM3->ARR = 0xFFFF;                         // set value of the autoload register to 65535, this is the highest value the counter can count to
  TIM3->DCR = 0;

  // setting up timer 4 to serve as the PWM output for the drone motors
  TIM_TypeDef *instanceTimer4 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pinGroupTimer3), PinMap_PWM);
  timer4 = new HardwareTimer(instanceTimer4);
  timer4->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PB6); // pin high when counter < channel compare, low otherwise
  timer4->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, PB7);
  timer4->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, PB8);
  timer4->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, PB9);
  TIM4->CR1 = TIM_CR1_CEN; // this enables the counter clock
  TIM4->CR2 = 0;
  TIM4->SMCR = 0; // this ensures the internal clock is used by the timer
  TIM4->DIER = 0; // since we will be using this as PWM, we dont need to enable interrupts on these pins
  TIM4->EGR = 0;
  TIM4->CCMR1 = 0b0110100001101000;                                           // setting OC2M and OC1M bits to 110, also setting OC2PE and OC1PE bits to 1
  TIM4->CCMR2 = 0b0110100001101000;                                           // setting OC2M and OC1M bits to 110, also setting OC2PE and OC1PE bits to 1
  TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; // configure the capture compare cannels to output rising edges
  TIM4->PSC = 71;                                                             // set prescaler value to 72 so timer updates at 1 Mhz
  TIM4->ARR = 5000;                                                           // set value of the autoload register to 65535, this is the highest value the counter can count to
  TIM4->DCR = 0;
  TIM4->CCR1 = 1000; // register value here determines the pulse length, a 1000 microsecond length here initializes the motors
  TIM4->CCR2 = 1000;
  TIM4->CCR3 = 1000;
  TIM4->CCR4 = 1000;
}

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

void startAltimeter()
{
  Wire1.beginTransmission(altimeterAddress);
  Wire1.write(0x1E); // Reset command
  Wire1.endTransmission();
  delay(250);
  int i;
  // getting calibration values from altimeter PROM, each "C" variable is 2 bytes in length, so we increment the address by 2
  for (i = 1; i <= 6; i++)
  {
    Wire1.beginTransmission(altimeterAddress);
    Wire1.write(0xA0 + i * 2);
    Wire1.endTransmission();
    Wire1.requestFrom(altimeterAddress, 2);
    while (Wire1.available() < 2)
      ;
    c[i - 1] = Wire1.read() << 8 | Wire1.read(); // c is a 16 bit integer
  }
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
  setupTimers();
  pinMode(PB3, OUTPUT);
  pinMode(PB4, OUTPUT);
  digitalWrite(PB4, LOW);
  Wire1.begin();
  delay(250);
  startGyro();
  startAltimeter();
  delay(250);
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

  // Serial.print("Channel 1: ");
  // Serial.print(rawReceiverPitch);
  // Serial.print(" ");
  // Serial.print("Channel 2: ");
  // Serial.println(rawReceiverRoll);
}
