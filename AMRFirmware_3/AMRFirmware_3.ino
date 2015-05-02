
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <PinChangeInt.h>
#include "TimerOne.h"
#include "RobotSerialComm.h"
#include "HardwareSerial.h"
#include "TimerOne.h"

boolean lockout = false;
boolean last = false;
boolean leftMotorOn = false;
boolean rightMotorOn = false;
boolean count = false;
boolean notRunning = true;

// Arguments for the reply
uint8_t reply_arg[5];

// Incoming message arguments
unsigned int arg[5];

// Timer count from velocity control
uint8_t TC = 0;
uint8_t TClast = 0;

// RobotSerial Communication
RobotSerialComm port;

//Initialize encoder variables
uint16_t rightCnt = 0;
uint16_t leftCnt = 0;

MPU6050 mpu;

#define LED_PIN 13
bool blinkState = false;

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
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw = 0;
float startYaw = 0;
float currentYaw = 0;
float targetYaw = 0;
float errorYaw = 0;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(460800);

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }

    //while(millis() < 15000) if (mpuInterrupt || fifoCount >= packetSize) yaw = getYaw();

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
    #define RWDIR 6
    #define LWDIR 5
    #define RWPWM 11
    #define LWPWM 3
    pinMode(RWDIR,OUTPUT);
    pinMode(LWDIR,OUTPUT);
    digitalWrite(RWDIR,0);
    digitalWrite(LWDIR,0);
    analogWrite(RWPWM,0);
    analogWrite(LWPWM,0);  
  
  
    //Encoder Initialization
    pinMode(8,INPUT);
    pinMode(9,INPUT);
    digitalWrite(8,HIGH); //Use internal pullup
    digitalWrite(9,HIGH); //Use internal pullup
    PCintPort::attachInterrupt(8, rightInt, CHANGE);
    PCintPort::attachInterrupt(9, leftInt, CHANGE);
  
    //Motion Detector Initialization
    pinMode(4,INPUT);
    digitalWrite(4,HIGH);
    PCintPort::attachInterrupt(4, mdetect, CHANGE);
  
    //Color Detector Initialization
    //Set up the color detector to detect red
    pinMode(12,OUTPUT);
    digitalWrite(12,LOW);
    pinMode(10,OUTPUT);
    digitalWrite(10,LOW);
    pinMode(7,INPUT);
  
    //Set Timer/Counter1 to generate an interrupt every 25ms
    Timer1.initialize(184320); //230400 cycles is 25ms, 184320 cycles is 20ms
    Timer1.attachInterrupt(timerIsr); // attach the service routine here    
}

boolean softStart(uint8_t dir,uint8_t dist,uint8_t kR,uint8_t kL,uint8_t kY,uint8_t initPWM){          //dir: 1-->left, 2-->right, 0-->straight dist: distance to travel (in encoder counts)
  notRunning = false;
  uint8_t LWPWR, RWPWR;
  int16_t pwmValLeft;
  int16_t pwmValRight;
  int8_t errorL;
  int8_t errorR;
  int16_t errorYawInt;
  int32_t mtradj;
  leftCnt = 0;
  rightCnt = 0;
  TC = 0;
  rightMotorOn = leftMotorOn = true;
  if(dir == 0){
    Timer1.initialize(184320);
    digitalWrite(RWDIR,0);
    digitalWrite(LWDIR,0);
    analogWrite(LWPWM, 0);
    analogWrite(RWPWM, 0);
    count = true;
    startYaw = yaw;
    while((rightCnt <= dist) && !lockout){
      if (mpuInterrupt || fifoCount >= packetSize){
        yaw = getYaw();
        if(150.0 < startYaw && startYaw < 180.0){
          if(yaw < 0.0) yaw = 360.0 - yaw;
        }
        else if(-180.0 < startYaw && startYaw < -150.0){
          if(yaw > 0.0) yaw = yaw - 360.0;
        }
        errorYaw = (yaw - startYaw)*kY;
        errorYawInt = errorYaw >= 0 ? (int16_t)(errorYaw+0.5) : (int16_t)(errorYaw-0.5);
      }
      //Veering left --> increasing Yaw, Veering right --> decreasing Yaw
      //errorL = TC - leftCnt;
      errorR = TC - rightCnt;
      pwmValLeft = initPWM + kL*errorL + errorYawInt;
      pwmValRight = initPWM + kR*errorR;
      if(pwmValLeft > 254) pwmValLeft = 254;
      else if(pwmValLeft < 1) pwmValLeft = 1;
      if(pwmValRight > 254) pwmValRight = 254;
      else if(pwmValLeft < 1) pwmValLeft = 1;
      LWPWR = pwmValLeft;
      RWPWR = pwmValRight;
      analogWrite(LWPWM, LWPWR);
      analogWrite(RWPWM, RWPWR);
      //reddetect();
    }
    count = false;
    analogWrite(LWPWM, 0);
    analogWrite(RWPWM, 0);
    rightMotorOn = leftMotorOn = false;
    if(lockout) return false;
    else return true;
  }    
  if(dir == 1){
    while (!mpuInterrupt && fifoCount < packetSize);
    yaw = getYaw();
    Timer1.initialize(400000);
    TC=0;
    digitalWrite(RWDIR, 0);
    digitalWrite(LWDIR, 1);
    analogWrite(RWPWM, 0);
    analogWrite(LWPWM, 255);
    count = true;
    startYaw = yaw;
    if(startYaw + 90 >= 180){                  //predict whether +/-180 degree discontinuity will take place
      startYaw = startYaw - 360;
      currentYaw = startYaw;
      targetYaw = startYaw + 90;
      //debug block
      /*Serial.print("discon\r\n");      
      Serial.print(startYaw);
      Serial.print("\r\n");
      Serial.print(currentYaw);
      Serial.print("\r\n");
      Serial.print(targetYaw);
      Serial.print("\r\n");*/
      while(1){
        if(mpuInterrupt || fifoCount >= packetSize){
          currentYaw = getYaw();
          /*Serial.print(currentYaw);
          Serial.print("\r\n");*/
          if(currentYaw > 0) currentYaw = currentYaw - 360;
          if(currentYaw >= targetYaw) break;
          mtradj = (90 + currentYaw - targetYaw) >= 0 ? (int32_t)((90 + currentYaw - targetYaw)+0.5) : (int32_t)((90 + currentYaw - targetYaw)-0.5);
          Timer1.initialize(400000 + mtradj*kY*10);
          errorR = TC - rightCnt;
          errorL = TC - leftCnt;
          pwmValLeft = 255 - initPWM - kL*errorL;
          pwmValRight = initPWM + kR*errorR;
          if(pwmValLeft > 254) pwmValLeft = 254;
          else if(pwmValLeft < 1) pwmValLeft = 1;
          if(pwmValRight > 254) pwmValRight = 254;
          else if(pwmValLeft < 1) pwmValLeft = 1;
          LWPWR = pwmValLeft;
          RWPWR = pwmValRight;
          analogWrite(LWPWM, LWPWR);
          analogWrite(RWPWM, RWPWR);
        }         
      }
    }
    else{
      //we know start yaw
      currentYaw = startYaw;
      targetYaw = startYaw + 90;
      //debug block
      /*Serial.print("cont\r\n");
      Serial.print(startYaw);
      Serial.print("\r\n");
      Serial.print(currentYaw);
      Serial.print("\r\n");
      Serial.print(targetYaw);
      Serial.print("\r\n");*/
      while(1){
        if(mpuInterrupt || fifoCount >= packetSize){
          currentYaw = getYaw();
          /*Serial.print(currentYaw);
          Serial.print("\r\n");*/
          if(currentYaw >= targetYaw) break;
          mtradj = (90 + currentYaw - targetYaw) >= 0 ? (int32_t)((90 + currentYaw - targetYaw)+0.5) : (int32_t)((90 + currentYaw - targetYaw)-0.5);
          Timer1.initialize(400000 + mtradj*kY*10);
          errorR = TC - rightCnt;
          errorL = TC - leftCnt;
          pwmValLeft = 255 - initPWM - kL*errorL;
          pwmValRight = initPWM + kR*errorR;
          if(pwmValLeft > 254) pwmValLeft = 254;
          else if(pwmValLeft < 1) pwmValLeft = 1;
          if(pwmValRight > 254) pwmValRight = 254;
          else if(pwmValLeft < 1) pwmValLeft = 1;
          LWPWR = pwmValLeft;
          RWPWR = pwmValRight;
          analogWrite(LWPWM, LWPWR);
          analogWrite(RWPWM, RWPWR);
        }
      }
    }
    count = false;
    analogWrite(LWPWM, 255);
    analogWrite(RWPWM, 0);
    rightMotorOn = leftMotorOn = false;
    return true;
  }
  if(dir == 2){
    while (!mpuInterrupt && fifoCount < packetSize);
    yaw = getYaw();
    Timer1.initialize(400000);
    TC=0;
    digitalWrite(RWDIR, 1);
    digitalWrite(LWDIR, 0);
    analogWrite(RWPWM, 255);
    analogWrite(LWPWM, 0);
    count = true;
    startYaw = yaw;
    if(startYaw - 90 <= -180){                  //predict whether +/-180 degree discontinuity will take place
      startYaw = startYaw + 360;
      currentYaw = startYaw;
      targetYaw = startYaw - 90;
      //debug block
      /*Serial.print("discon\r\n");      
      Serial.print(startYaw);
      Serial.print("\r\n");
      Serial.print(currentYaw);
      Serial.print("\r\n");
      Serial.print(targetYaw);
      Serial.print("\r\n");*/
      while(1){
        if(mpuInterrupt || fifoCount >= packetSize){
          currentYaw = getYaw();
          /*Serial.print(currentYaw);
          Serial.print("\r\n");*/
          if(currentYaw < 0) currentYaw = currentYaw + 360;
          if(currentYaw <= targetYaw) break;
          mtradj = (90 - currentYaw + targetYaw) >= 0 ? (int32_t)((90 - currentYaw + targetYaw)+0.5) : (int32_t)((90 - currentYaw + targetYaw)-0.5);
          Timer1.initialize(400000 + mtradj*kY*10);
          errorR = TC - rightCnt;
          errorL = TC - leftCnt;
          pwmValLeft = initPWM + kL*errorL;
          pwmValRight = 255 - initPWM - kR*errorR;
          if(pwmValLeft > 254) pwmValLeft = 254;
          else if(pwmValLeft < 1) pwmValLeft = 1;
          if(pwmValRight > 254) pwmValRight = 254;
          else if(pwmValLeft < 1) pwmValLeft = 1;
          LWPWR = pwmValLeft;
          RWPWR = pwmValRight;
          analogWrite(LWPWM, LWPWR);
          analogWrite(RWPWM, RWPWR);
        }
      }
    }
    else{
      //we know start yaw
      currentYaw = startYaw;
      targetYaw = startYaw - 90;
      //debug block
      /*Serial.print("cont\r\n");
      Serial.print(startYaw);
      Serial.print("\r\n");
      Serial.print(currentYaw);
      Serial.print("\r\n");
      Serial.print(targetYaw);
      Serial.print("\r\n");*/
      while(1){
        if(mpuInterrupt || fifoCount >= packetSize){
          currentYaw = getYaw();
          /*Serial.print(currentYaw);
          Serial.print("\r\n");*/
          if(currentYaw <= targetYaw) break;
          mtradj = (90 - currentYaw + targetYaw) >= 0 ? (int32_t)((90 - currentYaw + targetYaw)+0.5) : (int32_t)((90 - currentYaw + targetYaw)-0.5);
          Timer1.initialize(400000 + mtradj*kY*10);
          errorR = TC - rightCnt;
          errorL = TC - leftCnt;
          pwmValLeft = initPWM + kL*errorL;
          pwmValRight = 255 - initPWM - kR*errorR;
          if(pwmValLeft > 254) pwmValLeft = 254;
          else if(pwmValLeft < 1) pwmValLeft = 1;
          if(pwmValRight > 254) pwmValRight = 254;
          else if(pwmValLeft < 1) pwmValLeft = 1;
          LWPWR = pwmValLeft;
          RWPWR = pwmValRight;
          analogWrite(LWPWM, LWPWR);
          analogWrite(RWPWM, RWPWR);
        }
      }
    }
    count = false;
    analogWrite(LWPWM, 0);
    analogWrite(RWPWM, 255);
    rightMotorOn = leftMotorOn = false;
    return true;      
  }
}

float getYaw(){
  // reset interrupt flag and get INT_STATUS byte
  int16_t yawInt;
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      //Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
        
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      //yawInt = (ypr[0] * 180/M_PI) >= 0 ? (int16_t)((ypr[0] * 180/M_PI)+0.5) : (int16_t)((ypr[0] * 180/M_PI)-0.5);
      //Serial.print(yawInt);
      //Serial.print("\r\n");
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
      return (ypr[0] * 180/M_PI);
    }
}  

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    notRunning = true;

    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    
    if (mpuInterrupt || fifoCount >= packetSize) yaw = getYaw();
    
    int action = port.getMsg(arg);
    
    // If we got an action...Process it:
    switch(action){
    //Actions at this point will be:
    //1: DRIVE CMD @1,"direction","disth"e, reply: @1,"status"e  status: 0-->done with drive command, 1-->found object
            case 1:
                if(softStart(arg[0],arg[1],arg[2],arg[3],arg[4],arg[5])){
                  if(lockout) reply_arg[0] = 1;
                  else reply_arg[0] = 0;
                  port.reply(1, reply_arg, 1);
                }
                else{
                  if(lockout) reply_arg[0] = 1;
                  else reply_arg[0] = 0;
                  port.reply(1, reply_arg, 1);
                }
                break;

            default:
                break;
    }
}

void rightInt(){
 if(rightMotorOn){
   rightCnt++;    
 } 
}

void leftInt(){
 if(leftMotorOn){
   leftCnt++;
 }
}

void timerIsr(){
  if(count) TC++;
}

void mdetect(){
  if(notRunning) lockout = true;
}

void reddetect(){
  char i = 0;
  unsigned long freqr, freqg, freqb, freqc;
  char check = 0;
  while(i < 1){
    digitalWrite(12,LOW); //s2
    digitalWrite(10,LOW); //s3
    delay(5);
    freqr = pulseIn(7, HIGH);
    //Serial.write(freqr);
    digitalWrite(12,HIGH);
    delay(5);
    freqc = pulseIn(7, HIGH);
    //Serial.write(freqc);
    digitalWrite(10,HIGH);
    delay(5);
    freqg = pulseIn(7, HIGH);
    //Serial.write(freqg);
    digitalWrite(12,LOW);
    delay(5);
    freqb = pulseIn(7, HIGH);
    //Serial.write(freqb);
    if ((freqr > 0x19) && (freqr < 0x27)){
     if ((freqc > 0x08) && (freqc < 0x16)){
       if ((freqg > 0x35) && (freqg < 0x4A)){
         if ((freqb > 0x29) && (freqb < 0x36)){
           check++;
         }
         //else check = 0;
       }
       //else check = 0;
     }
     //else check = 0;
    }
    //else check = 0;
    i++;
    delay(1);
  }
  if (check >= 1){
    lockout = true;
  }
}
