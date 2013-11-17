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
const int inputLeft = A3;     //left  rssi input
const int inputRight = A1;    //right rssi input
const int inputUp= A2;     //left  rssi input
const int inputDown = A0;    //right rssi input
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
int panPos = 90;
int tiltPos = 180;

//max min pos
int maxTilt = 20;
int minTilt = 0;
int maxPan = 180;
int minPan = 0;
int snapPoint = 50;



//kalman filtering
double kalman_q= 0.02; //process noise covariance
double kalman_r= 500; //measurement noise covariance

void setup()
{ 
  Serial.begin(9600);
  servoPan.attach(servoPanPin);  
  servoTilt.attach(servoTiltPin); 
  smoothTilt(180); 
  smoothPan(90);  
}

void loop() {
 
  rssiLeft = kalman_rssiLeft_update(analogRead(inputLeft) + offsetLeft);
  rssiRight = kalman_rssiRight_update(analogRead(inputRight) + offsetRight);
  rssiUp = kalman_rssiUp_update(analogRead(inputUp) + offsetUp);
  rssiDown = kalman_rssiDown_update(analogRead(inputDown) + offsetDown);  
  
  int rssiPanDiff = rssiLeft > rssiRight ? rssiLeft - rssiRight : rssiRight - rssiLeft;
  int rssiTiltDiff = rssiUp > rssiDown ? rssiUp - rssiDown : rssiDown - rssiUp;
  
  
  //pan
  if(rssiPanDiff < 5){
    Serial.println('pan: centered');
  } else if(rssiLeft > rssiRight){
    Serial.println('pan: left');   
    int x = panPos + rssiPanDiff;
    if(x > maxPan){
       x = maxPan; 
    }  
    smoothPan(x); 
  } else if(rssiLeft < rssiRight){
    Serial.println('pan: right');   
    int x = panPos - rssiPanDiff;
    if(x > minPan){
       x = minPan; 
    }      
    smoothPan(x);    
  }  
  
  //tilt
  if(rssiTiltDiff < 5){
    Serial.println('tilt: centered');
  } else if(rssiUp > rssiDown){
    Serial.println('tilt: up');    
    int x = panPos + rssiTiltDiff;
    if(x > maxTilt){
       x = maxTilt; 
    }      
    smoothTilt(x); 
  } else if(rssiUp < rssiDown){
    Serial.println('tilt: down'); 
    int x = panPos - rssiTiltDiff;
    if(x > minTilt){
       x = minTilt; 
    }      
    smoothTilt(x);    
  }  
 
  //snapback
  if(panPos >= maxPan){
    if(rssiLeft > rssiRight && rssiPanDiff > snapPoint){
      Serial.println('snap: left');   
      int x = panPos / 2;
      smoothPan(x); 
    }
  }
  
 if(panPos <= minPan){
  if(rssiLeft < rssiRight && rssiPanDiff < snapPoint){
    Serial.println('snap: right');   
    int x = panPos * 2;  
    smoothPan(x);    
  }  
 } 
   
  
  
}

void smoothPan(int targetPos){
  int currentPos = panPos;
  int pos;
  int step;

  
  if(currentPos < targetPos){
      while(currentPos < targetPos){
              int step = constrain(currentPos > targetPos ? currentPos - targetPos : targetPos - currentPos,0,maxspeed); //dynamic servo speed  
              
              currentPos = currentPos + step; //reset pointer
              servoPan.write(step); //move the servo  
              delay(15);         //delay to allow servo to get to position
              
      }  
  } else if (targetPos < currentPos){
      while(targetPos < currentPos){
              int step = constrain(currentPos > targetPos ? currentPos - targetPos : targetPos - currentPos,0,maxspeed); //dynamic servo speed
              
              currentPos = currentPos - step; //reset pointer
              servoPan.write(step); //move the servo    
              delay(15);         //delay to allow servo to get to position
              
      }        
  } 
  panPos = currentPos;
}

void smoothTilt(int targetPos){
  int currentPos = panPos;
  int pos;
  int step;
  
  if(currentPos < targetPos){
      while(currentPos < targetPos){
              int step = constrain(currentPos > targetPos ? currentPos - targetPos : targetPos - currentPos,0,maxspeed); //dynamic servo speed 
              
              currentPos = currentPos + step; //reset pointer
              servoTilt.write(step); //move the servo  
              delay(15);         //delay to allow servo to get to position
              
      }  
  } else if (targetPos < currentPos){
      while(targetPos < currentPos){
              int step = constrain(currentPos > targetPos ? currentPos - targetPos : targetPos - currentPos,0,maxspeed); //dynamic servo speed
              
              
              currentPos = currentPos - step; //reset pointer
              servoTilt.write(step); //move the servo    
              delay(15);         //delay to allow servo to get to position
              
      }        
  } 
  panPos = currentPos;
}


float kalman_rssiLeft_update(float measurement)
{
  static int Llcnt=0;
  static float Lx=rssiLeft; //value
  static float Lp=100; //estimation error covariance
  static float Lk=0; //kalman gain  
  // update the prediction value
  Lp = Lp + kalman_q;

  // update based on measurement
  Lk = Lp / (Lp + kalman_r);
  Lx = Lx + Lk * (measurement - Lx);
  Lp = (1 - Lk) * Lp;
  
  return Lx;
}

float kalman_rssiRight_update(float measurement)
{
  static int Rlcnt=0;
  static float Rx=rssiRight; //value
  static float Rp=100; //estimation error covariance
  static float Rk=0; //kalman gain  
  // update the prediction value
  Rp = Rp + kalman_q;

  // update based on measurement
  Rk = Rp / (Rp + kalman_r);
  Rx = Rx + Rk * (measurement - Rx);
  Rp = (1 - Rk) * Rp;
  
  return Rx;
}


float kalman_rssiUp_update(float measurement)
{
  static int Ulcnt=0;
  static float Ux=rssiUp; //value
  static float Up=100; //estimation error covariance
  static float Uk=0; //kalman gain  
  // update the prediction value
  Up = Up + kalman_q;

  // update based on measurement
  Uk = Up / (Up + kalman_r);
  Ux = Ux + Uk * (measurement - Ux);
  Up = (1 - Uk) * Up;
  
  return Ux;
}


float kalman_rssiDown_update(float measurement)
{
  static int Dlcnt=0;
  static float Dx=rssiDown; //value
  static float Dp=100; //estimation error covariance
  static float Dk=0; //kalman gain  
  // update the prediction value
  Dp = Dp + kalman_q;

  // update based on measurement
  Dk = Dp / (Dp + kalman_r);
  Dx = Dx + Dk * (measurement - Dx);
  Dp = (1 - Dk) * Dp;
  
  return Dx;
}







