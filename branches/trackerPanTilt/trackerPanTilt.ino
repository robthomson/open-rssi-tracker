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
int offsetRight = 30;
int offsetUp = 14;
int offsetDown = 0;
int targetPos;
int smoothing = 5;

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
int snapPoint = 2;
int panSpeed;
int tiltSpeed;
int panStep = 1;
int tiltStep = 2;


void setup()
{ 
  Serial.begin(9600);
  Serial.println("Init..");
  servoPan.attach(servoPanPin);  
  servoTilt.attach(servoTiltPin);   
  servoPan.write(panPos);   
  servoTilt.write(tiltPos);   
  //calibrate();
  Serial.println("Tracking..");

}

void loop() {

/*
  Serial.println(offsetLeft);
  Serial.println(offsetRight);
  Serial.println(offsetUp);
  Serial.println(offsetDown);
  Serial.println("");
  */
  
  rssiLeft = analogRead(inputLeft) + offsetLeft;
  rssiRight = analogRead(inputRight) + offsetRight;
  rssiUp = analogRead(inputUp) + offsetUp;
  rssiDown = analogRead(inputDown) + offsetDown;  
  
  Serial.println(rssiLeft);
  Serial.println(rssiRight);

  int raw_rssiPanDiff = rssiLeft > rssiRight ? rssiLeft - rssiRight : rssiRight - rssiLeft;
  float averagePan_rssi = (rssiLeft + rssiRight ) / 2.f;
  const float K_rssiPan_ratio = 350.f; 
  int rssiPanDiff= (int)( (raw_rssiPanDiff * K_rssiPan_ratio) / averagePan_rssi ) ;

  int raw_rssiTiltDiff = rssiUp > rssiDown ? rssiUp - rssiDown : rssiDown - rssiUp;
  float averageTilt_rssi = (rssiUp + rssiDown ) / 2.f;
  const float K_rssiTilt_ratio = 350.f; 
  int rssiTiltDiff= (int)( (raw_rssiTiltDiff * K_rssiTilt_ratio) / averageTilt_rssi ) ;
  
  panPos = constrain(panPos,minPan,maxPan);
  tiltPos = constrain(tiltPos,minTilt,maxTilt);

  panSpeed = calcspeed(rssiPanDiff);
  tiltSpeed = calcspeed(rssiTiltDiff);

   if(rssiLeft < 130 && rssiRight < 130){
        //no lock do a sweep
        int pos = 0;
               for(pos = 0; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees 
                {                                  // in steps of 1 degree 
                  servoPan.write(pos);              // tell servo to go to position in variable 'pos' 
                  delay(20);                       // waits 15ms for the servo to reach the position 
                  
                  if(rssiLeft >= 130 || rssiRight >= 130){
                     break; 
                  }  
                  
                } 
                for(pos = 180; pos>=1; pos-=1)     // goes from 180 degrees to 0 degrees 
                {                                
                  servoPan.write(pos);              // tell servo to go to position in variable 'pos' 
                  delay(20);                       // waits 15ms for the servo to reach the position 
                  
                  if(rssiLeft >= 130 || rssiRight >= 130){
                     break; 
                  }
                  
                }         
     
      
   }  else {

 
        //left right
        if(rssiPanDiff <= smoothing){
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
      
        //up down
        if(rssiTiltDiff <= smoothing){
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


      //snap
        if(panPos <= minPan + 5 ){
            if(rssiRight > rssiLeft && rssiPanDiff >= snapPoint){
                 panPos = 160;                              
                 servoPan.write(panPos);              // tell servo to go to position in variable 'pos' 
                 delay(2000);                       // waits 15ms for the servo to reach the position         
            }    
        }  
        if(panPos >= maxPan - 5 ){
          if(rssiRight < rssiLeft && rssiPanDiff >= snapPoint){
                 panPos = 20;                              
                 servoPan.write(panPos);              // tell servo to go to position in variable 'pos' 
                 delay(2000);                       // waits 15ms for the servo to reach the position  
          }      
        }    
        
   }
  

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
    } else {
      speed = 10; 
    } 
   return speed; 
}  










