/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Modified by:   Andre Araujo & David Portugal on 04/01/2013
* Modified by:	 André Araújo & Nuno Ferreira  on 20/08/2013
* Version: Traxbot_Stingbot_DriverROS_v2
*********************************************************************/
// Arduino libraries
#include <EEPROM.h>
#include <stdlib.h>
#include <Wire.h>
#include <string.h>
#include <math.h>
#include <I2Cdev.h>
#include <PinChangeInt.h>
#include "MPU6050_6Axis_MotionApps20.h"


// RobotSerial Communication lib
#include "RobotSerialComm.h"
#include "HardwareSerial.h"

// TimerOne Library
#include "TimerOne.h"

// Set up MPU
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
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t yawh;
uint8_t yawl;
int yawint;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

boolean lockout = false;
boolean last = false;
boolean leftMotorOn = false;
boolean rightMotorOn = false;

// Arguments for the reply
uint8_t reply_arg[5];
unsigned int arg[5];

// RobotSerial Communication
RobotSerialComm port;

//Initialize encoder variables
unsigned int rightCnt = 0;
unsigned int leftCnt = 0;
char rwdirglob = 0; //used for correction
char lwdirglob = 0; //used for correction

//Sensor variables
char color = 0;
char motion = 0;

//I2C address of MPU6050
//const char address = 0x68;

// ******* Setup *******
void setup(){
  
  Wire.begin();
  TWBR = 30;
  
  //Status LED on sensor shield
  pinMode(13,OUTPUT);
  digitalWrite(13,0);
  
  // Serial port stuff
  Serial.begin(460800);
  
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    
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
  }
  
  //Initialize PWM here!!
  //Set DDRs for PWM output bits
  pinMode(6,OUTPUT);
  digitalWrite(6,0);
  pinMode(11,OUTPUT);
  digitalWrite(11,0);
  pinMode(5,OUTPUT);
  digitalWrite(5,0);
  pinMode(3,OUTPUT);
  digitalWrite(3,0);
  TCCR0A = 0x01;  //Phase Correct PWM Mode
  TCCR0B = 0x02;
  TCNT0 = 0;
  TCCR2A = 0x01;  //Phase Correct PWM Mode
  TCCR2B = 0x02;
  TCNT2 = 0;
  
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
}


// ******* Helper functions *******

//Need to create helper functions to read and process gyro, accelerometer, color detector, and IR motion detector

void motorDrive(char rwdir,char rwpwr,char lwdir, char lwpwr){
  if(rwpwr != 0){
   rightMotorOn = true;
  }   
  else{
   rightMotorOn = false;
   rightCnt = 0;
  }
  if(lwpwr != 0){
   leftMotorOn = true;
  }
  else{
    leftMotorOn = false;
    leftCnt = 0;
  }
  TCNT0 = 0;
  TCNT2 = 0;
  //Right wheel motor control
  if (lockout == false){
    if(rwdir==1){ //going forward
      rwdirglob = 1;
      TCCR0A &= ~0xF0;
      TCCR0A |= 0x20;
      OCR0B = rwpwr;  
    }
    else{ //going backward
      rwdirglob = 0;
      TCCR0A &= ~0xF0;
      TCCR0A |= 0x80;
      OCR0A = rwpwr;
    }
  //Left wheel motor control
    if(lwdir==1){ //going forward
      lwdirglob = 1;
      TCCR2A &= ~0xF0;
      TCCR2A |= 0x20;
      OCR2B = lwpwr;
    }
    else{ //going backward
      TCCR2A &= ~0xF0;
      lwdirglob = 0;
      TCCR2A |= 0x80;
      OCR2A = lwpwr;
    }
  }
}

void getCountData(){
  reply_arg[0] = rightCnt;
  reply_arg[1] = leftCnt;
  reply_arg[2] = yawh;
  reply_arg[4] = yawl;
  port.reply(2, reply_arg, 4);
}

void querySensorStatus(){
  reply_arg[0] = color;
  reply_arg[1] = motion;
  port.reply(2, reply_arg, 2);
}

/*void sendEncodersReads(){  
  
    reply_arg[0] = omni.read_enc3();
    reply_arg[1] = omni.read_enc1();
    port.reply(OMNI_READ_ENCODERS, reply_arg, 2);
}

void sendSonarsReads(){
  
    reply_arg[0] = robot.getRange(FRONT_SONAR);
    reply_arg[1] = robot.getRange(LEFT_SONAR);
    reply_arg[2] = robot.getRange(RIGHT_SONAR);
    port.reply(READ_SONARS, reply_arg, 3);
}*/


// ******* Main loop *******
void loop(){
      
    reddetect();
      
    int action = port.getMsg(arg);
      
    //Serial.write(action);
      
    digitalWrite(13,HIGH);
      
    // If we got an action...Process it:
    switch(action){
    //Actions at this point will be:
    //1: MOTOR_CONTROL @1,"rwdir","rwpwr","lwdir","lwpwr"e, no reply
    //2: GET_POSE_DATA @2e, reply: @2,"rightCnt","leftCnt","yaw"e
    //3: SENSOR_STATUS @3e, reply: @3,"color","motion"e
          case 1:
              motorDrive(arg[0],arg[1],arg[2],arg[3]);
              break;
            
          case 2:
              getCountData();
              break;
                
          case 3:
              querySensorStatus();
              break;

          default:
              break;
        
    } // switch
 
    if (mpuInterrupt && fifoCount > packetSize){
      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();

      // get current FIFO count
      fifoCount = mpu.getFIFOCount();

      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
          // reset so we can continue cleanly
          mpu.resetFIFO();
  
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
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
          yawint = ypr[0] >= 0 ? (int)(ypr[0]+0.5) : (int)(ypr[0]-0.5);
          if (yawint < 0) 360-yawint;
          yawh = yawint>>8;
          yawl = yawint&0x00FF;       
      }
    }   
} // loop()

void rightInt(){
 if(rightMotorOn){
   rightCnt++;
   if((leftCnt - rightCnt) > 3){
     if(rwdirglob == 1){
       if(128 < OCR0B < 135) OCR0B++;     //speed up the right wheel in the forward direction but impose a limit
     }
     else{
       if(128 < OCR0A < 135) OCR0A++;     //speed up the right wheel in the reverse direction but impose a limit
     }
   }
   if((rightCnt - leftCnt) > 3){
     if(rwdirglob == 1){
       if(128 > OCR0B > 121) OCR0B--;     //slow down the right wheel in the forward direction but impose a limit
     }
     else{
       if(128 > OCR0A > 121) OCR0A--;     //slow down the right wheel in the reverse direction but impose a limit
     }
   }     
 } 
}
//not going to implement correction for left wheel so right and left do not fight each other
void leftInt(){
 if(leftMotorOn){
   leftCnt++;
 }
}

void mdetect(){
  OCR0A = OCR0B = OCR2A = OCR2B = 0;
  motion = 1;
  lockout = true;
  //digitalWrite(13,1);
}


void reddetect(){
  char i = 0;
  unsigned long freqr, freqg, freqb, freqc;
  char check = 0;
  while(i <= 1){
    digitalWrite(12,LOW); //s2
    digitalWrite(10,LOW); //s3
    delay(1000);
    freqr = pulseIn(7, HIGH);
    //Serial.write(freqr);
    digitalWrite(12,HIGH);
    delay(1000);
    freqc = pulseIn(7, HIGH);
    //Serial.write(freqc);
    digitalWrite(10,HIGH);
    delay(1000);
    freqg = pulseIn(7, HIGH);
    //Serial.write(freqg);
    digitalWrite(12,LOW);
    delay(1000);
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
    delay(1000);
  }
  if (check > 1){
    OCR0A = OCR0B = OCR2A = OCR2B = 0;
    color = 1;
    lockout = true;
  }
  else color = 0;
}
    
// EOF









