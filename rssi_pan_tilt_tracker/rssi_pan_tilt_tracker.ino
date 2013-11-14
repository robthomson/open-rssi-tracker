/*
 * Authors 
 * - Rob Thomson <rob@marotori.com>
 
 * This tracker code was built to replace the analog tracker created by IBCrazy on
 * http://www.rcgroups.com/forums/showthread.php?t=1337608
 * 
 * The arduino is a 90% drop in replacement for the board.   The only difference.
 * with this system, simply use a propoer 360 degree servo.  Do not do the actual
 * mod required for the analog system.
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


const int servocenter = 90;


const int inputLeft = A3;     //left  rssi input
const int inputRight = A1;    //right rssi input
const int inputUp= A2;     //left  rssi input
const int inputDown = A0;    //right rssi input

const int servoPanPin = 11;
const int servoTiltPin = 12;

int panSpeed = 1;

int rssiLeft;
int rssiRight;
int rssiUp;
int rssiDown;
int calibrateLeft = 0;
int calibrateRight = 0;
int calibrateUp = 0;
int calibrateDown = 0;
int offsetLeft = 0;
int offsetRight = 19;
int offsetUp = 0;
int offsetDown = 0;

int panPos = 0;
int tiltPos = 0;

Servo servoPan;  
Servo servoTilt;  

int i=0;

void setup()
{ 
  Serial.begin(9600);
  servoPan.attach(servoPanPin);  
  servoTilt.attach(servoTiltPin); 

      
     panPos = servocenter;
     tiltPos = servocenter;
  
     servoTilt.write(servocenter + 90);  
     servoPan.write(servocenter);    
     
}

void loop() {
  
    

    rssiLeft = analogRead(inputLeft) + offsetLeft;
    rssiRight = analogRead(inputRight) + offsetRight;
    rssiUp = analogRead(inputUp) + offsetUp;
    rssiDown = analogRead(inputDown) + offsetDown;  
   
    int rssiPanDiff = rssiLeft > rssiRight ? rssiLeft - rssiRight : rssiRight - rssiLeft;
    int rssiTiltDiff = rssiUp > rssiDown ? rssiUp - rssiDown : rssiDown - rssiUp;


  
    if(rssiPanDiff < 5){
        servoPan.write(panPos);  
    } else {      
        if(rssiPanDiff < 10){
              panSpeed = 1;
        } else if(rssiPanDiff < 20 && rssiPanDiff >= 10){
              panSpeed = 2;
        } else if(rssiPanDiff < 30 && rssiPanDiff >= 20){
              panSpeed = 3;
        } else {
              panSpeed = 4;
        }  
      
      Serial.write(rssiPanDiff);
      
        if(rssiLeft > rssiRight){
          panPos = panPos + panSpeed;
        } else if(rssiLeft < rssiRight){
          panPos = panPos - panSpeed;
        } 
        servoPan.write(panPos);  
        delay(100);
    }      
      //Serial.println(panPos);


      

  
}  






