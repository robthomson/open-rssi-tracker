/*
 * Authors 
 * - Rob Thomson <rob@marotori.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <Servo.h>
#include <WProgram.h>
#include <EEPROM.h>

//hardware
const int inputLeft = A0;     //left  rssi input
const int inputRight = A1;    //right rssi input
const int inputUp= A2;     //left  rssi input
const int inputDown = A3;    //right rssi input
const int servoPanPin = 11;
const int servoTiltPin = 12;

//declare all variables
int rssiLeft;
int rssiRight;
int rssiUp;
int rssiDown;
int calibrateLeft = 0;
int calibrateRight = 0;
int calibrateUp = 0;
int calibrateDown = 0;
int offsetLeft = 0;
int offsetRight;
int offsetUp;
int offsetDown;
int targetPos;
int maxspeed = 4;

//init servo objects
Servo servoPan;  
Servo servoTilt;  

//initial servo positions
int panPos = 90;   //90 = center 0 = right ||  180 = left
int tiltPos = 180; //180 = straight ahead  || 80 = straight up || 60 = full back

//max min pos
int maxTilt = 180;
int minTilt = 60;
int maxPan = 180;
int minPan = 0;
int snapPoint = 10;
int panSpeed = 100;
int tiltSpeed = 100;
int panStep = 1;
int tiltStep = 2;

//kalman filtering
double kalman_q= 0.02; //process noise covariance
double kalman_r= 500; //measurement noise covariance

void setup()
{ 
  Serial.begin(9600);
  servoPan.attach(servoPanPin);  
  servoTilt.attach(servoTiltPin); 
  
 
  
  servoPan.write(panPos);   
  servoTilt.write(tiltPos);  
  
  calibrate();

  Serial.println("Init..");
}

void loop() {



  rssiLeft = analogRead(inputLeft) + offsetLeft;
  rssiRight = analogRead(inputRight) + offsetRight;
  rssiUp = analogRead(inputUp) + offsetUp;
  rssiDown = analogRead(inputDown) + offsetDown;  

  int rssiPanDiff = rssiLeft > rssiRight ? rssiLeft - rssiRight : rssiRight - rssiLeft;
  int rssiTiltDiff = rssiUp > rssiDown ? rssiUp - rssiDown : rssiDown - rssiUp;

  if(rssiPanDiff <= 10){
       panStep = 1; 
  } else if(rssiPanDiff <= 15){
      panStep = 2; 
  } else if(rssiPanDiff <= 20){
      panStep = 3; 
  }  else {
      panStep = 4;    
  }  
  
  if(rssiTiltDiff <= 10){
       tiltStep = 1; 
  } else if(rssiPanDiff <= 15){
      tiltStep = 2; 
  } else if(rssiPanDiff <= 20){
      tiltStep = 3; 
  }   else {
      tiltStep = 4;    
  }  

  panPos = constrain(panPos,minPan,maxPan);
  tiltPos = constrain(tiltPos,minTilt,maxTilt);

  if(rssiPanDiff <= 5){
    Serial.println("center");
        servoPan.write(panPos);
  } else if(rssiLeft >= rssiRight){
        Serial.println("move right");
        panPos = panPos + panStep;
        servoPan.write(panPos);
        delay(panSpeed);      
  } else if(rssiLeft <= rssiRight){
        Serial.println("move left");
        panPos = panPos - panStep;
        servoPan.write(panPos);
        delay(panSpeed);      
  } 



  if(rssiTiltDiff <= 10){
        Serial.println("center");
        servoTilt.write(tiltPos);
  } else if(rssiUp >= rssiDown){
        Serial.println("move up");
        tiltPos = tiltPos + tiltStep;
        servoTilt.write(tiltPos);
        delay(tiltSpeed);      
  } else if(rssiUp <= rssiDown){
        Serial.println("move down");
        tiltPos = tiltPos - tiltStep;
        servoTilt.write(tiltPos);
        delay(tiltSpeed);      
  } 
  



  

}

void calibrate(){

  Serial.println("Calibrate..");
  int i;
  for(i=0;i<10;i++)
  {
    calibrateLeft = calibrateLeft + analogRead(inputLeft);
    calibrateRight = calibrateRight + analogRead(inputRight);
    calibrateUp = calibrateUp + analogRead(inputUp);
    calibrateDown = calibrateDown + analogRead(inputDown);  
    delay(25);
  }
  calibrateLeft=calibrateLeft/10;  
  calibrateRight=calibrateRight/10;  
  calibrateUp=calibrateUp/10;  
  calibrateDown=calibrateDown/10;    

  int diffPanRSSI = calibrateLeft > calibrateRight ? calibrateLeft - calibrateRight : calibrateRight - calibrateLeft;
  int diffTiltRSSI = calibrateUp > calibrateDown ? calibrateUp - calibrateDown : calibrateDown - calibrateUp;

  if(calibrateLeft >= calibrateRight){
    offsetLeft = 0;
    offsetRight = diffPanRSSI;        

  } 
  else {
    offsetLeft = diffPanRSSI;
    offsetRight = 0; 
  } 

  if(calibrateUp >= calibrateDown){
    offsetUp = 0;
    offsetDown = diffTiltRSSI;        

  } 
  else {
    offsetUp = diffTiltRSSI;
    offsetDown = 0; 
  }    


}  












