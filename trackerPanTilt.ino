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
int offsetRight = 0;
int offsetUp = 0;
int offsetDown = 0;
int targetPos;
int smoothing = 10;

//init servo objects
Servo servoPan;  
Servo servoTilt;  

//initial servo positions
int panPos = 90;   //90 = center 0 = right ||  180 = left
int tiltPos = 175; //175 = straight ahead  || 80 = straight up || 60 = full back

//max min pos
int maxTilt = 180;
int minTilt = 60;
int maxPan = 180;
int minPan = 0;
int snapPoint = 5;
int panSpeed;
int tiltSpeed;
int panStep = 1;
int tiltStep = 2;

double kalman_q= 0.02; //process noise covariance
double kalman_r= 100; //measurement noise covariance
int kalman_p=100; //eastimated error covariance

void setup()
{ 
  Serial.begin(9600);
  Serial.println("Init..");
  servoPan.attach(servoPanPin);  
  servoTilt.attach(servoTiltPin);   
  pan(panPos);   
  tilt(tiltPos);   
  delay(5000);
  calibrate();
  Serial.println("Tracking..");

}

void loop() {

      track();


}

void track(){
 

  rssiLeft = kalman_rssiLeft_update(analogRead(inputLeft) + offsetLeft);
  rssiRight = kalman_rssiRight_update(analogRead(inputRight) + offsetRight);
  rssiUp = kalman_rssiUp_update(analogRead(inputUp) + offsetUp);
  rssiDown = kalman_rssiDown_update(analogRead(inputDown) + offsetDown);  



  int raw_rssiPanDiff = rssiLeft > rssiRight ? rssiLeft - rssiRight : rssiRight - rssiLeft;
  float averagePan_rssi = (rssiLeft + rssiRight ) / 2.f;
  const float K_rssiPan_ratio = 400.f; 
  int rssiPanDiff= (int)( (raw_rssiPanDiff * K_rssiPan_ratio) / averagePan_rssi ) ;

  int raw_rssiTiltDiff = rssiUp > rssiDown ? rssiUp - rssiDown : rssiDown - rssiUp;
  float averageTilt_rssi = (rssiUp + rssiDown ) / 2.f;
  const float K_rssiTilt_ratio = 400.f; 
  int rssiTiltDiff= (int)( (raw_rssiTiltDiff * K_rssiTilt_ratio) / averageTilt_rssi ) ;

  panPos = constrain(panPos,minPan,maxPan);
  tiltPos = constrain(tiltPos,minTilt,maxTilt);

  panSpeed = calcspeed(rssiPanDiff);
  tiltSpeed = calcspeed(rssiTiltDiff);


    if (rssiLeft < 120 || rssiRight <120){
        //lost signal so hold position.
        Serial.println("lost signal - hold pan");
        pan(panPos);
        delay(panSpeed);
    } else if(rssiPanDiff <= smoothing){
      Serial.println("center");
      pan(panPos);
      delay(panSpeed);
    } 
    else if(rssiLeft >= rssiRight){
      Serial.println("move right");
      panPos = panPos + panStep;
      pan(panPos);
      delay(panSpeed);      
    } 
    else if(rssiLeft <= rssiRight){
      Serial.println("move left");
      panPos = panPos - panStep;
      servoPan.write(panPos);
      delay(panSpeed);      
    } 

    //up down
     if (rssiUp < 120 || rssiDown <120){
        //lost signal so hold position.
        Serial.println("lost signal - hold tilt");
        tilt(tiltPos);
        delay(panSpeed);
    } else if(rssiTiltDiff <= smoothing){
      Serial.println("center");
      tilt(tiltPos);
      delay(panSpeed);
    } 
    else if(rssiUp >= rssiDown){
      Serial.println("move up");
      tiltPos = tiltPos + tiltStep;
      tilt(tiltPos);
      delay(tiltSpeed);      
    } 
    else if(rssiUp <= rssiDown){
      Serial.println("move down");
      tiltPos = tiltPos - tiltStep;
      tilt(tiltPos);
      delay(tiltSpeed);      
    } 


    //snap
    if(panPos <= minPan + 5 ){
      if(rssiRight > rssiLeft && rssiPanDiff >= snapPoint){
        panPos = 160;                              
        pan(panPos);              // tell servo to go to position in variable 'pos' 
        delay(2000);                       // waits 15ms for the servo to reach the position         
      }    
    }  
    if(panPos >= maxPan - 5 ){
      if(rssiRight < rssiLeft && rssiPanDiff >= snapPoint){
        panPos = 20;                              
        pan(panPos);              // tell servo to go to position in variable 'pos' 
        delay(2000);                       // waits 15ms for the servo to reach the position  
      }      
    }    

  
}  

void tilt(int tgt){
     servoTilt.write(tgt); 
}  

void pan(int tgt){
     servoPan.write(tgt); 
}  

void calibrate(){

  Serial.println("Calibrate..");
  delay(2000);
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


float kalman_rssiLeft_update(float measurement)
{
  static int Llcnt=0;
  static float Lx=rssiLeft; //value
  static float Lp=kalman_p; //estimation error covariance
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
  static float Rp=kalman_p; //estimation error covariance
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
  static float Up=kalman_p; //estimation error covariance
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
  static float Dp=kalman_p; //estimation error covariance
  static float Dk=0; //kalman gain  
  // update the prediction value
  Dp = Dp + kalman_q;

  // update based on measurement
  Dk = Dp / (Dp + kalman_r);
  Dx = Dx + Dk * (measurement - Dx);
  Dp = (1 - Dk) * Dp;
  
  return Dx;
}

int calcspeed(int rssiDiff){
  int speed;
  if(rssiDiff <= smoothing){
    speed = 0;
  } 
  else if (rssiDiff <= smoothing + 1){
    speed = 200 ;
  } 
  else if (rssiDiff <= smoothing + 2){
    speed = 150 ;
  } 
  else if (rssiDiff <= smoothing + 4){
    speed =  100;
  } 
  else if (rssiDiff <= smoothing + 6){
    speed = 50 ;
  } 
  else if (rssiDiff <= smoothing + 8){
    speed = 30 ;         
  } 
  else {
    speed = 10; 
  } 
  return speed; 
}  











