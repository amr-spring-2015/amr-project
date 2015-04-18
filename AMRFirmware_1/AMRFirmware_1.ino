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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <PinChangeInt.h>

// RobotSerial Communication lib
#include "RobotSerialComm.h"
#include "HardwareSerial.h"

// TimerOne Library
#include "TimerOne.h"

boolean lockout = false;
boolean last = false;
boolean leftMotorOn = false;
boolean rightMotorOn = false;
boolean count = false;

// Arguments for the reply
uint8_t reply_arg[5];
unsigned int arg[5];

// Timer count from velocity control
uint8_t TC = 0;
uint8_t TClast = 0;

// RobotSerial Communication
RobotSerialComm port;

//Initialize encoder variables
uint16_t rightCnt = 0;
uint16_t leftCnt = 0;

// ******* Setup *******
void setup(){
  
  //Status LED on sensor shield
  pinMode(13,OUTPUT);
  digitalWrite(13,0);
  
  // Serial port stuff
  Serial.begin(460800);
  
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
  
  //Set Timer/Counter1 to generate an interrupt every 20ms
  Timer1.initialize(20000); //20ms is actually hard coded into TimerOne.cpp because normal function wouldn't work with 18.432MHz crystal
  Timer1.attachInterrupt(timerIsr); // attach the service routine here
}


// ******* Helper functions *******

//Need to create helper functions to read and process gyro, accelerometer, color detector, and IR motion detector

boolean softStart(uint8_t dir,uint8_t dist){          //dir: 1-->left, 2-->right, 0-->straight dist: distance to travel (in encoder counts)
  uint8_t LWPWR, RWPWR;
  uint8_t kL = 4;
  uint8_t kR = 4;
  int16_t pwmValLeft;
  int16_t pwmValRight;
  int8_t errorL;
  int8_t errorR;
  leftCnt = 0;
  rightCnt = 0;
  TC = 0;
  rightMotorOn = leftMotorOn = true;
  if(dir == 0){
    digitalWrite(RWDIR,0);
    digitalWrite(LWDIR,0);
    analogWrite(LWPWM, 0);
    analogWrite(RWPWM, 0);
    count = true;
    while((TC <= dist) && !lockout/* && !reddetect()*/){
      errorL = TC - leftCnt;
      errorR = TC - rightCnt;
      pwmValLeft = 70 + kL*errorL;
      pwmValRight = 75 + kR*errorR;
      if(pwmValLeft > 255) pwmValLeft = 255;
      else if(pwmValLeft < 0) pwmValLeft = 0;
      if(pwmValRight > 255) pwmValRight = 255;
      else if(pwmValLeft < 0) pwmValLeft = 0;
      LWPWR = pwmValLeft;
      RWPWR = pwmValRight;
      analogWrite(LWPWM, LWPWR);
      analogWrite(RWPWM, RWPWR);
    }
    count = false;
    analogWrite(LWPWM, 0);
    analogWrite(RWPWM, 0);
    rightMotorOn = leftMotorOn = false;
    if(TC < dist) return false;
    else return true;
  }    
    

  /*if(dir == 1){
    digitalWrite(RWDIR, 0);
    digitalWrite(LWDIR, 1);
    RWPWR = 0;
    LWPWR = 255;
  }
  if(dir == 2){
    digitalWrite(RWDIR, 1);
    digitalWrite(LWDIR, 0);
    RWPWR = 255;
    LWPWR = 0;
  }*/
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
  
    int action = port.getMsg(arg);
    
    // If we got an action...Process it:
    switch(action){
    //Actions at this point will be:
    //1: DRIVE CMD @1,"direction","disth"e, reply: @1,"status"e  status: 0-->done with drive command, 1-->found object
            case 1:
                if(softStart(arg[0],arg[1])){
                  reply_arg[0] = 0;
                  port.reply(1, reply_arg, 1);
                }
                else{
                  reply_arg[0] = 1;
                  port.reply(1, reply_arg, 1);
                }
                break;

            default:
                break;
        
   } // switch
   /*if (last == false){
     digitalWrite(13,1);
     last = true;
   }
   else{
     digitalWrite(13,0);
     last = false;
   }*/
    //delay(1000);
} // loop()

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
  //lockout = true;
}

boolean reddetect(){
  char i = 0;
  unsigned long freqr, freqg, freqb, freqc;
  char check = 0;
  while(i <= 1){
    digitalWrite(12,LOW); //s2
    digitalWrite(10,LOW); //s3
    delay(1);
    freqr = pulseIn(7, HIGH);
    //Serial.write(freqr);
    digitalWrite(12,HIGH);
    delay(1);
    freqc = pulseIn(7, HIGH);
    //Serial.write(freqc);
    digitalWrite(10,HIGH);
    delay(1);
    freqg = pulseIn(7, HIGH);
    //Serial.write(freqg);
    digitalWrite(12,LOW);
    delay(1);
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
  if (check > 1){
    return false;
  }
  else return false;
}
    
// EOF









