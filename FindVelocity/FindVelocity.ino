#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <PinChangeInt.h>

uint16_t rightCnt = 0;
uint16_t leftCnt = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(460800);
  //Encoder Initialization
  pinMode(8,INPUT);
  pinMode(9,INPUT);
  digitalWrite(8,HIGH); //Use internal pullup
  digitalWrite(9,HIGH); //Use internal pullup
  PCintPort::attachInterrupt(8, rightInt, CHANGE);
  PCintPort::attachInterrupt(9, leftInt, CHANGE);
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
}
//Answer 54 --> interrupt every 20 ms
void loop() {
  // put your main code here, to run repeatedly:
 analogWrite(RWPWM, 80);
 analogWrite(LWPWM, 80);
 delay(1000);
 analogWrite(RWPWM, 0);
 analogWrite(LWPWM, 0);
 Serial.print(rightCnt);
 while(1);  
}

void rightInt(){
  rightCnt++;
}

void leftInt(){
  leftCnt++;
}
