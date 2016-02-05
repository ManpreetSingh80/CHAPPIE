#define version_0.66

//#define DEBUGING            //uncomment it to debug.It will decrease the processing speed.
#define BLUE                //comment if not using bluetooth

//#include <EEPROMex.h>                             //EEPROM library for storing pid values
#include <I2Cdev.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>
#include <PID_v1.h>                              //Arduino PID library
#include <digitalIOPerformance.h>                //library for faster pin R/W
#define DIGITALIO_NO_INTERRUPT_SAFETY
#define DIGITALIO_NO_MIX_ANALOGWRITE

//Bluetooth Stuff
#include<SoftwareSerial.h>

const int rxpin = 11; // pin used to receive (not used in this version) 
const int txpin = 4; // pin used to send to LCD

SoftwareSerial blue(rxpin, txpin); // new serial port on pins 2 and 3

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
#define BALANCE_KP 15                   // PID Constants
#define BALANCE_KI 90
#define BALANCE_KD 0.8
#define BALANCE_PID_MIN -255              // Define PID limits to match PWM max in reverse and foward
#define BALANCE_PID_MAX 255

#define ROTATION_KP 50
#define ROTATION_KI 300
#define ROTATION_KD 4

#define MOTOR_A_DIR      5         //M11
#define MOTOR_A_BRAKE    8         //M12
#define MOTOR_B_DIR      6         //M21
#define MOTOR_B_BRAKE    12        //M22
#define MOTOR_A_PWM      9         //M1E
#define MOTOR_B_PWM      10        //M2E

// Motor Misc
#define PWM_MIN 0
#define PWM_MAX 255
float MOTORSLACK_A=32;                     // Compensate for motor slack range (low PWM values which result in no motor engagement)
float MOTORSLACK_B=39;                     // Compensate for motor slack range (low PWM values which result in no motor engagement)
#define MOTOR_A_PWM_MAX 255               // Compensate for differences in DC motor strength
#define MOTOR_B_PWM_MAX 255               

int MotorAspeed, MotorBspeed, MotorSlack,moveState=0,d_speed,d_dir;

double yaw,input,out,setpoint,originalSetpoint,Buffer[3];
double yinput,yout,ysetpoint,yoriginalSetpoint;
//uint32_t timer,timer1;

double bal_kp,bal_ki,bal_kd,rot_kp,rot_ki,rot_kd;
//int addressFloat=0;

PID pid(&input,&out,&setpoint,BALANCE_KP,BALANCE_KI,BALANCE_KD,DIRECT);
PID rot(&yinput,&yout,&ysetpoint,ROTATION_KP,ROTATION_KI,ROTATION_KD,DIRECT);

String content = "";
  char character;
//String d;



void setup()
{
  #ifdef DEBUGING
  Serial.begin(9600);
  #endif
  
  #ifdef BLUE
  blue.begin(9600);
  blue.setTimeout(10);
  #endif
  
  init_imu();
  initmot();
  
pid.SetMode(AUTOMATIC);                  //For info about these,see Arduino PID library
pid.SetOutputLimits(-210, 210);
pid.SetSampleTime(10);
rot.SetMode(AUTOMATIC);
rot.SetOutputLimits(-20, 20);
rot.SetSampleTime(10);

setpoint = 0;
originalSetpoint = setpoint;
ysetpoint = 0;
yoriginalSetpoint = ysetpoint;

//first save pid values in EEPROM manually,otherwise it will give garbage values
/*
addressFloat  = EEPROM.getAddress(sizeof(float));          //reading from EEPROM
bal_kp=EEPROM.readFloat(addressFloat);
bal_ki=EEPROM.readFloat((addressFloat+=sizeof(float)));
bal_kd=EEPROM.readFloat((addressFloat+=sizeof(float)));
rot_kp=EEPROM.readFloat((addressFloat+=sizeof(float)));
rot_ki=EEPROM.readFloat((addressFloat+=sizeof(float)));
rot_kd=EEPROM.readFloat((addressFloat+=sizeof(float)));
*/

bal_kp=BALANCE_KP;
bal_ki=BALANCE_KI;
bal_kd=BALANCE_KD;
rot_kp=ROTATION_KP;
rot_ki=ROTATION_KI;
rot_kd=ROTATION_KD;

pid.SetTunings(bal_kp,bal_ki,bal_kd);                      //change PID values
rot.SetTunings(rot_kp,rot_ki,rot_kd);
}



void loop() 
{
  getvalues();        //read values from imu
  new_pid();          //call pid

#ifdef BLUE
Bt_control();          //App control
#endif

#ifdef DEBUGING
  printval();
#endif

}
    

void init_imu()
{
      // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
     mpu.initialize();
     devStatus = mpu.dmpInitialize();
     // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-9);
    mpu.setYGyroOffset(-3);
    mpu.setZGyroOffset(61);
    mpu.setXAccelOffset(-449);
    mpu.setYAccelOffset(2580);
    mpu.setZAccelOffset(1259);
    // make sure it worked (returns 0 if so)
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
    

void printval()
{
  Serial.print(yinput);Serial.print("\t"); 
  Serial.print(yoriginalSetpoint); Serial.print("\t");
  Serial.print(ysetpoint); Serial.print("\t");
  Serial.print(yout); Serial.print("\t");Serial.print("\t");  
  Serial.print(input);Serial.print("\t");
  Serial.print(originalSetpoint); Serial.print("\t");
  Serial.print(setpoint); Serial.print("\t");
  Serial.print(out); Serial.print("\t");Serial.print("\t");
  Serial.print(MotorAspeed); Serial.print("\t");
  Serial.print(MotorBspeed); Serial.println("\t");

}


void Bt_control()
{
if(blue.available())
  {
   content=blue.readString();
     if(content[0]=='F')
     setpoint = originalSetpoint - d_speed;//Serial.println(setpoint);}            //forward
     else if(content[0]=='B')
     setpoint = originalSetpoint + d_speed;//Serial.println(setpoint);}            //backward
     else if(content[0]=='L')
     ysetpoint = constrain((ysetpoint + yoriginalSetpoint - d_dir),-180,180);//Serial.println(ysetpoint);}      //left
     else if(content[0]=='R')
     ysetpoint = constrain(ysetpoint + yoriginalSetpoint + d_dir,-180,180);//Serial.println(ysetpoint);}        //right
     else if(content[0]=='S')
     d_speed = (content.substring(2)).toInt();//Serial.println(d_speed);}            //set speed slider
     else if(content[0]=='D')
     d_dir = content.substring(2).toInt();//Serial.println(d_dir);}                  //set direction slider
    else if(content[0]=='P')
    {
     if(content[1]=='S')
    {
     if(content[2]=='B')
     save_pid(1);                      //save balance pid values
     else
     save_pid(0);                      //save rotation pid values
    } else if(content[1]=='C')
    {
     if(content[2]=='B')
    {
      change_pid(1);                //change balance pid values
    }
    else
    change_pid(0);                //change rotation pid values
    }
    }
    if(content=="updateb")  
    return_pid(1);              //return balance pid values from eeprom
    else if(content=="updater")
    return_pid(0);              //return rotation pid values from eeprom
/*if(ysetpoint!=0)
{
  di="D";di.concat(ysetpoint);di.concat("e");blue.print(di);
Serial.println(di);}*/
  }
}



void return_pid(bool b)    //return values from eeprom
{
  char charVal[10];
  String sent = "";
  if(b)
  {
  sent.concat("OP");dtostrf(bal_kp, 5, 3, charVal);sent.concat(charVal);
  sent.concat("OI");dtostrf(bal_ki, 5, 3, charVal);sent.concat(charVal);
  sent.concat("OD");dtostrf(bal_kd, 5, 3, charVal);sent.concat(charVal);
  sent.concat("e");
 // Serial.println(sent);
  }
  else
  {
    sent.concat("OP");dtostrf(rot_kp, 5, 3, charVal);sent.concat(charVal);
  sent.concat("OI");dtostrf(rot_ki, 5, 3, charVal);sent.concat(charVal);
  sent.concat("OD");dtostrf(rot_kd, 5, 3, charVal);sent.concat(charVal);
  sent.concat("e");Serial.println(sent);
  }
 blue.print(sent);
}



void change_pid(bool b)        //change pid values
{
  blue.print("O");
  while(!blue.available());
  for(int i=0;i<3;i++)
  {
    Buffer[i]=blue.parseFloat();
  }
  if(b)
  {
    bal_kp=Buffer[0];bal_ki=Buffer[1];bal_kd=Buffer[2];
  //  Serial.print("Bkp");Serial.print(bal_kp);Serial.print("\t");
  //  Serial.print("Bki");Serial.print(bal_ki);Serial.print("\t");
  //  Serial.print("Bkd");Serial.print(bal_kd);Serial.println("\t");
  }
  else
  {
   rot_kp=Buffer[0];rot_ki=Buffer[1];rot_kd=Buffer[2];
 // Serial.print("Rkp");Serial.print(rot_kp);Serial.print("\t");
  //  Serial.print("Rki");Serial.print(rot_ki);Serial.print("\t");
  //  Serial.print("Rkd");Serial.print(rot_kd);Serial.println("\t"); 
  }
}



void save_pid(bool pid)        //save pid values
{ /* 
  addressFloat = 0;
 if(pid)
 {
   EEPROM.updateFloat(addressFloat,bal_kp);
 EEPROM.updateFloat((addressFloat+=sizeof(float)),bal_ki);
 EEPROM.updateFloat((addressFloat+=sizeof(float)),bal_kd);
  // Serial.println("Bupdated");
 }
 else
 {
   EEPROM.updateFloat(addressFloat,rot_kp);
 EEPROM.updateFloat((addressFloat+=sizeof(float)),rot_ki);
 EEPROM.updateFloat((addressFloat+=sizeof(float)),rot_kd);
   //Serial.println("Rupdated");
 }*/
}




double compensate_slack(double yOutput,double Output,bool A)
  {
   // Compensate for DC motor non-linear "dead" zone around 0 where small values don't result in movement
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
   // Convert PID output to motor control
   
     MotorAspeed = compensate_slack(yout,out,1);
     MotorBspeed = compensate_slack(yout,out,0);
     motorspeed(MotorAspeed, MotorBspeed);            //change speed
}
//Fast digitalWrite is implemented

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
