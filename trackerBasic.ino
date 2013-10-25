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
#include "MovingAvarageFilter.h"

const int inputLeft = A0;     //left  rssi input
const int inputRight = A1;    //right rssi input
const int inputMidFront = A3;  //front rssi input
const int inputMidTop = A2;    //top rssi input
const int buttonPin = 13;
const int outputServo = 10;    //pin of servo used for pan motion
const float smoothing = 5;  //

int EEsize = 1024; // size in bytes of your board's EEPROM

float rssiLeftAvg = 0;
float rssiRightAvg = 0;
float rssiMidFrontAvg = 0;
float rssiMidTopAvg = 0;


const int servocenter = 90;  // absolute center. a 'home made' 360 will no doubt need a suitable value to be found.
int speed = 0;
int hasSignal = 0;
int offsetLeft = 0;
int offsetRight = 0;
int buttonState = 0; 

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
  pinMode(buttonPin, INPUT); 

  int offsetLeft = EEPROM.read(1);
  int offsetRight = EEPROM.read(2);
}

void loop() {

  buttonState = digitalRead(buttonPin);

  int inCalibration = 0;
  while (buttonState == HIGH) { 
    inCalibration = 1;

    Serial.println("CALIBRATION MODE");
    servo.write(servocenter); 


    rssiLeftAvg = rssiLeftAverageFilter.process(analogRead(inputLeft)); 
    rssiRightAvg = rssiRightAverageFilter.process(analogRead(inputRight)); 
    int rssiLeft = rssiLeftAvg;
    int rssiRight = rssiRightAvg;


    int diffRSSI = rssiLeft > rssiRight ? rssiLeft - rssiRight : rssiRight - rssiLeft;

    if(rssiLeft > rssiRight){
      offsetLeft = 0;
      offsetRight = diffRSSI;        

    } 
    else {
      offsetLeft = diffRSSI;
      offsetRight = 0; 
    }  

    buttonState = digitalRead(buttonPin);

    Serial.println(rssiLeft);
    Serial.println(rssiRight);
    Serial.println(diffRSSI);
    Serial.println(offsetLeft);
    Serial.println(offsetRight);  

  } 
  if(inCalibration == 1){
    Serial.println("Writing epprom....");
    EEPROM.write(1,offsetLeft);
    EEPROM.write(2,offsetRight);
    inCalibration = 0; 
    delay(5000);
  }  



  Serial.println("SEEK MODE");


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



  if(rssiMidTop == 0 && rssiMidFront == 0){
    Serial.println("USB");
    servo.write(servocenter); 
    delay(1000);
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

}






