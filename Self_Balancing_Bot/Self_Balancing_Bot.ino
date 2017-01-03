//#define bluetooth

#include<I2Cdev.h>
#include<Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>
#include <PID_v1.h>                              //Arduino PID library
#include <digitalIOPerformance.h>                //library for faster pin R/W
#define DIGITALIO_NO_INTERRUPT_SAFETY
#define DIGITALIO_NO_MIX_ANALOGWRITE
#include<SoftwareSerial.h>

int txpin = 10; // bluetooth tx to 10 pin
int rxpin = 11; // bluetooth rx to 11 pin

SoftwareSerial bluetooth(txpin,rxpin);

#define RESTRICT_PITCH

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

#define BALANCE_KP 15                   // PID Constants
#define BALANCE_KI 90
#define BALANCE_KD 0.8
#define BALANCE_PID_MIN -255
#define BALANCE_PID_MAX 255

#define MOTOR_A_DIR      5         //M11
#define MOTOR_A_BRAKE    8         //M12
#define MOTOR_B_DIR      6         //M21
#define MOTOR_B_BRAKE    12        //M22
#define MOTOR_A_PWM      9         //M1E
#define MOTOR_B_PWM      3        //M2E

// Motor Misc
#define PWM_MIN 0
#define PWM_MAX 255
float MOTORSLACK_A=40;                     // Compensate for motor slack range (low PWM values which result in no motor engagement)
float MOTORSLACK_B=45;                     // Compensate for motor slack range (low PWM values which result in no motor engagement)
#define MOTOR_A_PWM_MAX 255               // Compensate for differences in DC motor strength
#define MOTOR_B_PWM_MAX 255               

int MotorAspeed, MotorBspeed, MotorSlack,moveState=0,d_speed,d_dir;

double yaw,input,out,setpoint,originalSetpoint,Buffer[3];

double bal_kp,bal_ki,bal_kd;

PID pid(&input,&out,&setpoint,BALANCE_KP,BALANCE_KI,BALANCE_KD,DIRECT);

void setup()
{
  
  bluetooth.begin(9600);
  bluetooth.setTimeout(10);
  
  //Serial.begin(9600);
  

  init_imu();
  initmot();

pid.SetMode(AUTOMATIC);                  //For info about these,see Arduino PID library
pid.SetOutputLimits(-210, 210);
pid.SetSampleTime(10);

bal_kp=BALANCE_KP;
bal_ki=BALANCE_KI;
bal_kd=BALANCE_KD;
    setpoint=0;
    originalSetpoint=setpoint;
    pid.SetTunings(bal_kp,bal_ki,bal_kd);
}

void loop(){
getvalues();
new_pid();
Bt_control();          //App control

//serialprint();
}

void serialprint(){
  Serial.print(input);Serial.print("\t");
  Serial.print(out); Serial.print("\t");Serial.print("\t");
  Serial.print(MotorAspeed); Serial.print("\t");
  Serial.print(MotorBspeed); Serial.println("\t");
  /*Serial.print(bal_kp); Serial.println("\t");
  Serial.print(bal_ki); Serial.println("\t");
  Serial.print(bal_kd); Serial.println("\t");*/}

void init_imu(){
    Wire.begin();
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
       // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}

void getvalues(){
  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
         } 
         input =-ypr[2] * 180/M_PI;          //change sign if negative
}
void initmot()
{
  //Pin definitions
    pinMode(MOTOR_A_DIR, OUTPUT);
    pinMode(MOTOR_A_BRAKE, OUTPUT);
    pinMode(MOTOR_B_DIR, OUTPUT);
    pinMode(MOTOR_B_BRAKE, OUTPUT);
    analogWrite(MOTOR_A_PWM, 0);
    analogWrite(MOTOR_B_PWM, 0);
}

double compensate_slack(double Output,bool A)
  {
   // Compensate for DC motor non-linear "dead" zone around 0 where small values don't result in movement
   //yOutput is for left,right control
  if(A)
  {
   if (Output >= 0) 
   Output = Output + MOTORSLACK_A ;
   if (Output < 0) 
   Output = Output - MOTORSLACK_A;
  }
  else
  {
    if (Output >= 0) 
   Output = Output + MOTORSLACK_B;
   if (Output < 0) 
   Output = Output - MOTORSLACK_B;
  }
   Output = constrain(Output, BALANCE_PID_MIN, BALANCE_PID_MAX); 
  return Output; 
}

void new_pid()
{
  //Compute error
  pid.Compute();
   // Convert PID output to motor control
   
     MotorAspeed = compensate_slack(out,1);
     MotorBspeed = compensate_slack(out,0);
     motorspeed(MotorAspeed, MotorBspeed);            //change speed
}
void motorspeed(int MotorAspeed, int MotorBspeed) {
  // Motor A control
  if (MotorAspeed >= 0) 
  {
    digitalWrite(MOTOR_A_DIR,HIGH);
    digitalWrite(MOTOR_A_BRAKE,LOW);
  }
  else 
{
  digitalWrite(MOTOR_A_DIR,LOW);
  digitalWrite(MOTOR_A_BRAKE,HIGH);
}
  
  analogWrite(MOTOR_A_PWM,abs(MotorAspeed));

  // Motor B control
  if (MotorBspeed >= 0) 
  {
    digitalWrite(MOTOR_B_DIR,HIGH);
    digitalWrite(MOTOR_B_BRAKE,LOW);
  }
  else 
  {
  digitalWrite(MOTOR_B_DIR,LOW);
  digitalWrite(MOTOR_B_BRAKE,HIGH);
  }
  analogWrite(MOTOR_B_PWM, abs(MotorBspeed));
}

void Bt_control(){
  while(bluetooth.available()>= 2 ){}
  
    unsigned int k = bluetooth.read();
    unsigned int k1 = bluetooth.read();
    unsigned int realK = (k1 *256) + k;

    if (realK >= 1000 && realK <1255) 
    {
      bal_kp = realK - 1000;
      Serial.print("Kp = ");
      Serial.println(bal_kp);
      delay(10);
    }
    if (realK >= 2000 && realK <2255) 
    {
      bal_ki = realK - 2000;
      Serial.print("Ki = ");
      Serial.println(bal_ki);
      delay(10);
    }
    if (realK >= 3000 && realK <3255) 
    {
      bal_kd = realK - 3000;
      Serial.print("Kd = ");
      Serial.println(bal_kd);
      delay(10);
    }
  
}

