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

// RobotSerial Communication lib
#include "RobotSerialComm.h"
#include "HardwareSerial.h"

// TimerOne Library
#include "TimerOne.h"

// IMU Library
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Arguments for the reply
int16_t reply_arg[5];

unsigned int arg[5];

// RobotSerial Communication
RobotSerialComm port;

//Streaming without interrupt:
boolean stream1=false;
boolean stream2=false;
boolean stream3=false;
boolean stream4=false;

// ******* Setup *******
void setup(){
  
  pinMode(13,OUTPUT);
  
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
  
  //Initialize Gyro/Accelerometer here!!
  
  //Initialize Color detector here!!
  
  //Initialize IR Motion sensor here!!
  
  //Set Timer/Counter1 to generate an interrupt every 10ms (timing is a guess) to go get gyro data
  Timer1.initialize(10000); //10ms is actually hard coded into TimerOne.cpp because normal function wouldn't work with 18.432MHz crystal
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here
}


// ******* Helper functions *******

//Need to create helper functions to read and process gyro, accelerometer, color detector, and IR motion detector

void motorDrive(char rwdir,char rwpwr,char lwdir, char lwpwr){
  TCNT0 = 0;
  TCNT2 = 0;
  if(rwdir==1){ //going forward
    TCCR0A &= ~0xF0;
    TCCR0A |= 0x20;
    OCR0B = rwpwr;  
  }
  else{ //going backward
    TCCR0A &= ~0xF0;
    TCCR0A |= 0x80;
    OCR0A = rwpwr;
  }
  
  if(lwdir==1){ //going forward
    TCCR2A &= ~0xF0;
    TCCR2A |= 0x20;
    OCR2B = lwpwr;
  }
  else{ //going backward
    TCCR2A &= ~0xF0;
    TCCR2A |= 0x80;
    OCR2A = lwpwr;
  }
}

/*void getAccelData(){
  reply_arg[0] = ax;
  reply_arg[1] = ay;
  reply_arg[2] = gz;
  port.reply(10, reply_arg, 3);
}*/

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
    //Need to define the streams here for this particular robot's communications:
    //1: Send x position, y position, and z angle  
    //2: Send sensor feedback; color detected, motion detected, future sensors
    //3: Test stream
    //4: Other (future)

    int action = port.getMsg(arg);
  
    if(action==0 && stream1==true){
      action=2;
    } 
 
    if(action==0 && stream2==true){
      action=3;
    }   
    
    if(action==0 && stream3==true){
      action=10;
    }
    
    
    // If we got an action...Process it:
    switch(action){
    //Actions at this point will be:
    //1: MOTOR_CONTROL @1,"rwdir","rwpwr","lwdir","lwpwr"e, no reply
    //2: GET_POSE_DATA @2e, reply: @2,"xpos","ypos","angz"e
    //3: GET_SENSOR_DATA @3e, reply: @3,"color","motion_flap"e
    //4: START_POSE_STREAM @4e, no reply
    //5: STOP_POSE STREAM @5e, no reply
    //6: START_SENSOR_STREAM @6e, no reply
    //7: STOP_SENSOR_STREAM @7e, no reply
    //8: START_TEST_STREAM @8e, no reply
    //9: STOP_TEST_STREAM @9e, no reply
    //10: RAW_ACCEL_DATA @10e, reply: @10,"ax_h","ax_l","ay_h","ay_l"e
            case 1:
                motorDrive(arg[0],arg[1],arg[2],arg[3]);
                break;
            
            case 2:
                break;
                
            case 3:
                break;

            case 4:
                stream1 = true;
                break;

            case 5:
                stream1 = false;
                break;

            case 6:
                stream2 = true;
                break;

            case 7:
                stream2 = false;
                break;    

            case 8:
                stream3 = true;
                break;    
                
            case 9:
                stream3 = false;             
                break;

            case 10:
                //getAccelData();       
                break;

            default:
                break;

        
   } // switch
    //delay(1000);
} // loop()

void timerIsr()
{
    
}
// EOF









