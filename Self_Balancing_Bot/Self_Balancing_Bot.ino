#include <Wire.h>

const int motorLeftSpeed = 6;
const int motorLeftDirection1 = 7;
const int motorLeftDirection2 = 8;
const int motorRightSpeed = 3;
const int motorRightDirection1 = 4;
const int motorRightDirection2 = 5;


long accelX, accelY, accelZ;
double gForceX, gForceY, gForceZ;
double angleAccelXY, angleAccelYZ, angleAccelZX, totalAccelAngle;

long gyroX, gyroY, gyroZ;
double rotX, rotY, rotZ;
double angleGyroXY, angleGyroYZ, angleGyroZX, totalGyroAngle;

double currentError = 0, prevError = 0;
long Kp, Ki, Kd;
double prop = 0, integral = 0, der = 0, input=0;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
}

void setupMPU()
{
  //Setting up the MPU
  Wire.beginTransmission(0b1101000); //This is the I2C address of MPU 6050
  Wire.write(0x6B); //Accessing the register 6B - Power Management
  Wire.write(0b00000000); //Setting SLEEP register to 0
  Wire.endTransmission();

  //Initializing Accelerometer
  Wire.beginTransmission(0b01101000); //I2C address of MPU
  Wire.write(0x1C); //Accessing the register 1C - Accelerometer configurartion
  Wire.write(0x00000000); //Setting the accel to +/- 2g
  Wire.endTransmission();

  //Initia;izing Gyroscope
  Wire.beginTransmission(0b1101000); //I2C address of MPU
  Wire.write(0x1B); //Accessing theregister 1B - Gyroscope configuration
  Wire.write(0x00000000);//Setting the gyro to full scale +/- 250 deg/sec
  Wire.endTransmission();
}


void loop()
{
  recordAccelRegister();
  recordGyroRegister();
  getAngle();
  PID();

  if(input > 0)
    motorSetupA();
  else
    motorSetupB();
}


void recordAccelRegister()
{
  Wire.beginTransmission(0b1101000); //I2C address of MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request Accel Registers
  while(Wire.available() < 6);
  accelX = Wire.read() << 8|Wire.read(); //Store first two bytes
  accelY = Wire.read() << 8|Wire.read(); // Store middle two bytes
  accelZ = Wire.read() << 8|Wire.read(); //Store last two bytes
  processAccelData;
}

void processAccelData()
{
  gForceX = accelX/13684.0;
  gForceY = accelY/13684.0;
  gForceZ = accelZ/13684.0;
}

void recordGyroRegister()
{
  Wire.beginTransmission(0b1101000); //I2C address of MPU
  Wire.write(0x43); //Starting register for gyro
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request gyro Registers
  while(Wire.available() < 6);
  gyroX = Wire.read() << 8|Wire.read(); //Store first two bytes
  gyroY = Wire.read() << 8|Wire.read(); //Store middle two bytes
  gyroZ = Wire.read() << 8|Wire.read(); //Store last two bytes
  processGyroData();
}

void processGyroData()
{
  rotX = gyroX/131.0;
  rotY = gyroY/131.0;
  rotZ = gyroZ/131.0;
}

void getAngle()
{
  angleAccelXY = atan2(gForceY, gForceX);
  angleAccelYZ = atan2(gForceZ, gForceY);
  angleAccelZX = atan2(gForceX, gForceZ);
  totalAccelAngle = angleAccelXY + angleAccelYZ + angleAccelZX;

  angleGyroXY = atan2(gyroY, gyroX);
  angleGyroYZ = atan2(gyroZ, gyroY);
  angleGyroZX = atan2(gyroX, gyroZ);
  totalGyroAngle = angleGyroXY + angleGyroYZ + angleGyroZX;
}

void PID()
{
  currentError = 0.98*(totalAccelAngle) + 0.02*(totalGyroAngle);
  prop = currentError;
  integral += prevError;
  der = currentError - prevError;

  input = Kp*(prop) + Ki*(integral) + Kd*(der);
}

void motorSetupA()
{
  if(input > 255)
    input = 255;

  analogWrite(motorLeftSpeed, input);
  digitalWrite(motorLeftDirection1, HIGH);
  digitalWrite(motorLeftDirection2, LOW);
  analogWrite(motorRightSpeed, input);
  digitalWrite(motorRightDirection1, HIGH);
  digitalWrite(motorRightDirection2, LOW);  
}

void motorSetupB()
{
  if(input < -255)
    input = -255;
  if(input < 0)
    input = -input;

  analogWrite(motorLeftSpeed, input);
  digitalWrite(motorLeftDirection1, LOW);
  digitalWrite(motorLeftDirection2, HIGH);
  analogWrite(motorRightSpeed, input);
  digitalWrite(motorRightDirection1, LOW);
  digitalWrite(motorRightDirection2, HIGH);  
}
