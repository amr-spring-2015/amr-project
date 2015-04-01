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
//#include <Wire.h> Cant use Wire.h because it uses interrupts for I2C communication and I want to call the MPU6050 from within an interrupt
#include <string.h>
#include <math.h>
//#include <I2C.h>
#include <PinChangeInt.h>


// RobotSerial Communication lib
#include "RobotSerialComm.h"
#include "HardwareSerial.h"

// TimerOne Library
#include "TimerOne.h"

// IMU Library
/*uint8_t axh, axl, gzh, gzl;
long ax = 0;
long axLast = 0;
long vx = 0;
long vxLast = 0;
long dx = 0;
long omegaz = 0;
long omegazLast = 0;
long thetaz = 0;
long zeroAx = 0;
long zeroOmegaz = 0;
long deltavx = 0;
long deltadx = 0;
uint8_t dx4, dx3, dx2, dx1;*/

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
  
  //Status LED on sensor shield
  pinMode(13,OUTPUT);
  digitalWrite(13,0);
  
  // Serial port stuff
  Serial.begin(460800);
    
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
  
  //Initialize Gyro/Accelerometer here!!
  /*I2c.begin();
  I2c.setSpeed(1);
  I2c.write(address, 0x6B, 0);  //Wake the MPU6050 up
  I2c.write(address, 0x1A, 0x04); //set the digital LPF to 21Hz
  delay(500000); 
  //Get initial value of ax and omegaz
  int i = 100;
  while (i>0){
    I2c.read(address, 0x3B, 2);
    zeroAx += I2c.receive()<<8|I2c.receive();
    i--;
  }
  zeroAx /= 100; 
  zeroAx &= 0xFFFFFFFE;*/
  
  //Set Timer/Counter1 to generate an interrupt every 10ms (timing is a guess) to go get gyro data
  //Timer1.initialize(100000); //100ms is actually hard coded into TimerOne.cpp because normal function wouldn't work with 18.432MHz crystal
  //Timer1.attachInterrupt( timerIsr ); // attach the service routine here 
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
  port.reply(2, reply_arg, 2);
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
    
    // If we got an action...Process it:
    switch(action){
    //Actions at this point will be:
    //1: MOTOR_CONTROL @1,"rwdir","rwpwr","lwdir","lwpwr"e, no reply
    //2: GET_POSE_DATA @2e, reply: @2,"rightCnt","leftCnt"e
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
    //delay(1000);
} // loop()

/*void timerIsr()
{
  if (last == false){
    digitalWrite(13,1);
    last = true;
  }
  else{
    digitalWrite(13,0);
    last = false;
  }
  if (findDistance==true){
    I2c.read(address, 0x3B, 2);
    axh = I2c.receive();
    axl = I2c.receive();
    ax = axh<<8|axl;
    ax &= 0xFFFFFFFE;
    ax = ax-zeroAx;                           //Zero Out
    vx = vx + ax*10;
    dx = dx + (vx*10);
    //dx = vx*10 + 0.5*ax*100;
    dx4 = (dx>>24) & 0xFF;
    dx3 = (dx>>16) & 0xFF;
    dx2 = (dx>>8) & 0xFF;
    dx1 = dx & 0xFF;
  }
}*/

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









