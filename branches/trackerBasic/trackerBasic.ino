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
#include "MovingAvarageFilter.h"

const int inputLeft = A0;     //left  rssi input
const int inputRight = A1;    //right rssi input
const int inputMidFront = A3;  //front rssi input
const int inputMidTop = A2;    //top rssi input
const int outputServo = 10;    //pin of servo used for pan motion
const float smoothing = 5;  //

float rssiLeftAvg = 0;
float rssiRightAvg = 0;
float rssiMidFrontAvg = 0;
float rssiMidTopAvg = 0;


const int servocenter = 90;  // absolute center. a 'home made' 360 will no doubt need a suitable value to be found.
int speed = 0;
int hasSignal = 0;
int offsetLeft = 10;
int offsetRight = 0;


/*
  probably no need to edit below here.
 */

Servo servo;  // create servo object to control a servo
MovingAvarageFilter rssiLeftAverageFilter(smoothing);
MovingAvarageFilter rssiRightAverageFilter(smoothing);
MovingAvarageFilter rssiMidTopAverageFilter(smoothing);
MovingAvarageFilter rssiMidFrontAverageFilter(smoothing);


void setup()
{ 
  Serial.begin(9600);
  servo.attach(outputServo);  
  
}

void loop() {


 
  
  rssiLeftAvg = rssiLeftAverageFilter.process(analogRead(inputLeft) + offsetLeft); 
  rssiRightAvg = rssiRightAverageFilter.process(analogRead(inputRight) + offsetRight); 
  rssiMidFrontAvg = rssiMidFrontAverageFilter.process(analogRead(inputMidFront) ); 
  rssiMidTopAvg = rssiMidTopAverageFilter.process(analogRead(inputMidTop)); 
  
  int rssiLeft = rssiLeftAvg;
  int rssiRight = rssiRightAvg;
  int rssiMidFront = rssiMidFrontAvg;
  int rssiMidTop = rssiMidTopAvg;

  int raw_rssiDiff = rssiLeft > rssiRight ? rssiLeft - rssiRight : rssiRight - rssiLeft;
  float average_rssi = (rssiLeft + rssiRight ) / 2.f;
  const float K_rssi_ratio = 500.f; 
  int rssiDiff = (int)( (raw_rssiDiff * K_rssi_ratio) / average_rssi ) ;

 Serial.println(rssiDiff);
 Serial.println(" ");

  if(rssiMidTop == 0 && rssiMidFront == 0){
    Serial.println("USB");
    servo.write(servocenter); 
  } else if ((rssiLeft < 120 || rssiRight <120)){
    if(hasSignal == 0){
      Serial.println("seeking 360");  
      servo.write(servocenter +  20); 
      delay(250);
      servo.write(servocenter);     
      delay(1000);            
    } 
    else {
      Serial.println("hold");
      servo.write(servocenter);
      delay(10000);    
      hasSignal = 0;          
    } 
  } 
  else {  
    hasSignal = 1; 
    speed = rssiDiff;
    //track
    if(rssiLeft > rssiRight) { 
      Serial.println("move left");   
      servo.write(servocenter +  speed);               
    }
    else if(rssiRight > rssiLeft) { 
      Serial.println("move right");  
      servo.write(servocenter -  speed);
    } 
    else {
      Serial.println("center");  
      servo.write(servocenter);
      delay(2000);
    }  
}
}


