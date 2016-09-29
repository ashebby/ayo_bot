#include <I2Cdev.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>
#include <PID_v1.h>                              //Arduino PID library
#include <digitalIOPerformance.h>                //library for faster pin R/W
#define DIGITALIO_NO_INTERRUPT_SAFETY
#define DIGITALIO_NO_MIX_ANALOGWRITE

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

// Balance PID controller Definitions
#define BALANCE_KP 100     
#define BALANCE_KI 1000           
#define BALANCE_KD 8
#define BALANCE_PID_MIN -255              // Define PID limits to match PWM max in reverse and foward
#define BALANCE_PID_MAX 255

#define ROTATION_KP 50.4376950551922  // 50
#define ROTATION_KI 500.195586913476  // 300
#define ROTATION_KD 4.66968339454473 //4

#define MOTOR_A_DIR      5         //6
#define MOTOR_A_BRAKE    8         //12
#define MOTOR_B_DIR      6         //M21
#define MOTOR_B_BRAKE    12        //M22
#define MOTOR_A_PWM      9         //M1E
#define MOTOR_B_PWM      10        //M2E

// Encoder
#define encodPinA1      3                       // encoder A pin
#define encodPinB1      4                       // encoder B pin

#define LOOPTIME        100                     // PID loop time
#define FORWARD         1                       // direction of rotation
#define BACKWARD        2                       // direction of rotation

unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing
long count = 0;                                 // rotation counter
long countInit;
long tickNumber = 0;
boolean run = false;                                     // motor moves

// Motor Misc
#define PWM_MIN 0
#define PWM_MAX 255
float MOTORSLACK_A=100;                 // 32 Compensate for motor slack range (low PWM values which result in no motor engagement)
float MOTORSLACK_B=0;                     // Compensate for motor slack range (low PWM values which result in no motor engagement)
#define MOTOR_A_PWM_MAX 255               // Compensate for differences in DC motor strength
#define MOTOR_B_PWM_MAX 255               

int MotorAspeed, MotorBspeed, MotorSlack,moveState=0,d_speed,d_dir;

double yaw,input,out,setpoint,originalSetpoint,Buffer[3];
double yinput,yout,ysetpoint,yoriginalSetpoint;

double bal_kp,bal_ki,bal_kd,rot_kp,rot_ki,rot_kd;

PID pid(&input,&out,&setpoint,BALANCE_KP,BALANCE_KI,BALANCE_KD,DIRECT);
PID rot(&yinput,&yout,&ysetpoint,ROTATION_KP,ROTATION_KI,ROTATION_KD,DIRECT);

String content = "";
  char character;

void setup()
{
  #ifdef DEBUGING
  Serial.begin(115200);
  #endif
  
  init_imu();
  
  pinMode(MOTOR_A_DIR, OUTPUT);
  pinMode(MOTOR_A_BRAKE, OUTPUT);
  pinMode(MOTOR_B_DIR, OUTPUT);
  pinMode(MOTOR_B_BRAKE, OUTPUT);
  pinMode(encodPinA1, INPUT);
  pinMode(encodPinB1, INPUT);
  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);
  digitalWrite(encodPinA1, HIGH);
  digitalWrite(encodPinB1, HIGH);
  
  pid.SetMode(AUTOMATIC);                  //For info about these,see Arduino PID library
  pid.SetOutputLimits(-8000, 8000);       // PID controller output default range is 0-255
  pid.SetSampleTime(10);
  rot.SetMode(AUTOMATIC);
  rot.SetOutputLimits(-50, 50);
  rot.SetSampleTime(10);

  setpoint = 1;
  originalSetpoint = setpoint;
  ysetpoint = 0;
  yoriginalSetpoint = ysetpoint;
  
  
  bal_kp=BALANCE_KP;
  bal_ki=BALANCE_KI;
  bal_kd=BALANCE_KD;
  rot_kp=ROTATION_KP;
  rot_ki=ROTATION_KI;
  rot_kd=ROTATION_KD;
  
  /* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
  pid.SetTunings(bal_kp,bal_ki,bal_kd);                      //change PID values
  rot.SetTunings(rot_kp,rot_ki,rot_kd);
  
}

void loop() 
{
  getvalues();        //read values from imu
  new_pid();          //call pid
}

void init_imu()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(30);
  mpu.setYGyroOffset(-9);
  mpu.setZGyroOffset(31);
  mpu.setXAccelOffset(-2702);
  mpu.setYAccelOffset(-1235);
  mpu.setZAccelOffset(1154);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {      // turn on the DMP, now that it's ready
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
    
void getvalues()
{
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
         yinput = ypr[0]* 180/M_PI;
         input = -ypr[1] * 180/M_PI;          //change sign if negative
}



double compensate_slack(double yOutput,double Output,bool A)
 {
  //Compensate for DC motor non-linear "dead" zone around 0 where small values don't result in movement
  //yOutput is for left,right control
  if(A)
  {
   if (Output >= 0) 
   Output = Output + MOTORSLACK_A - yOutput;
   if (Output < 0) 
   Output = Output - MOTORSLACK_A - yOutput;
  }
  else
  {
    if (Output >= 0) 
   Output = Output + MOTORSLACK_B + yOutput;
   if (Output < 0) 
   Output = Output - MOTORSLACK_B + yOutput;
  }
   Output = constrain(Output, BALANCE_PID_MIN, BALANCE_PID_MAX); 
  return Output;
}

void new_pid()
{
  //Compute error
  pid.Compute();
  rot.Compute();
  //Convert PID output to motor control
  MotorAspeed = compensate_slack(yout,out,1);
  MotorBspeed = compensate_slack(yout,out,0);
  motorspeed(MotorAspeed, MotorBspeed);      //change speed
}

// Motor control functions
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
    digitalWrite(MOTOR_B_DIR,LOW);
    digitalWrite(MOTOR_B_BRAKE,HIGH);
  }
  else 
  {
  digitalWrite(MOTOR_B_DIR,HIGH);
  digitalWrite(MOTOR_B_BRAKE,LOW);
  }
  analogWrite(MOTOR_B_PWM, abs(MotorBspeed));
}

