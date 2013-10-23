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

const int inputLeft = A0;     //left  rssi input
const int inputRight = A1;    //right rssi input
const int inputMidFront = A3;  //front rssi input
const int inputMidTop = A2;    //top rssi input
const int outputServo = 10;    //pin of servo used for pan motion
const float RSSIsmoothing = 0.0010;  // .0001 = max | 1 = 0ff

int counter = 0;

const int servocenter = 90;  // absolute center. a 'home made' 360 will no doubt need a suitable value to be found.
int speed = 0;
int hasSignal = 0;
/* 
 these offsets can be used to find center the rssi as most modules are not exact to each other!
 and easy way to find the exact values is to print the rssiLeft & rssiRight values - when the power
 to the rf modules is turned off - well. that worked for me!
 */
int inputLeftOffset = 17;  
int inputRightOffset = 0; 

/*
  probably no need to edit below here.
 */

Servo servo;  // create servo object to control a servo


void setup()
{ 
  Serial.begin(9600);
  servo.attach(outputServo);  
}

void loop() {

  
  float rssiLeftRough = analogRead(inputLeft) + inputLeftOffset ;
  float rssiRightRough = analogRead(inputRight) + inputRightOffset ;
  float rssiMidFrontRough = analogRead(inputMidFront);
  float rssiMidTopRough = analogRead(inputMidTop);

  int rssiLeft = smooth(rssiLeftRough, RSSIsmoothing, rssiLeft);  
  int rssiRight = smooth(rssiRightRough, RSSIsmoothing, rssiRight);  
  int rssiMidFront = smooth(rssiMidFrontRough, RSSIsmoothing, rssiMidFront); 
  int rssiMidTop = smooth(rssiMidTopRough, RSSIsmoothing, rssiMidTop); 




  int raw_rssiDiff = rssiLeft > rssiRight ? rssiLeft - rssiRight : rssiRight - rssiLeft;
  float average_rssi = (rssiLeft + rssiRight ) / 2.f;
  // Try setting this constant to the average raw rssi value where you are getting good results currently
  const float K_rssi_ratio = 500.f; 

  int rssiDiff = (int)( (raw_rssiDiff * K_rssi_ratio) / average_rssi ) ;
  
  
// Serial.println(rssiLeft);
// Serial.println(rssiRight);
// Serial.println(rssiMidFront);
//  Serial.println(rssiMidTop);
//  Serial.println(" ");
 Serial.println(rssiDiff);
  Serial.println(" ");
  /*
    ----------------------------------------------------------------------------- 
   TRACKING LOGIC
   ----------------------------------------------------------------------------- 
   */


  if(rssiMidTop == 0 && rssiMidFront == 0){
    Serial.println("USB");
    servo.write(servocenter);  
    delay(2000);
  }
  else if ((rssiLeft < 120 || rssiRight <120)){
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
  //end
}

int smooth(int data, float filterVal, float smoothedVal){


  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return (int)smoothedVal;
}

