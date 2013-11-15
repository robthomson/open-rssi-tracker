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

const int inputLeft = A0;     //left  rssi input
const int inputRight = A1;    //right rssi input
const int inputMidFront = A3;  //front rssi input
const int inputMidTop = A2;    //top rssi input
const int buttonPin = 13;
const int outputServo = 10;    //pin of servo used for pan motion
const int maxspeed = 40; // maximum speed of servo 

int EEsize = 1024; // size in bytes of your board's EEPROM

int rssiLeft;
int rssiRight;
int rssiMidTop;
int rssiMidFront;

const int servocenter = 90;  // absolute center. a 'home made' 360 will no doubt need a suitable value to be found.
int speed = 0;
int hasSignal = 0;
int offsetLeft = 0;
int offsetRight = 0;
int buttonState = 0; 
int smoothing = 6;
int centerhold = 10;
int lostlockhold = 5000;

double kalman_q= 0.02; //process noise covariance
double kalman_r= 500; //measurement noise covariance

/*
  probably no need to edit below here.
 */

Servo servo;  // create servo object to control a servo



void setup()
{ 
  Serial.begin(9600);
  servo.attach(outputServo);  
  pinMode(buttonPin, INPUT); 
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

void loop() {

  buttonState = digitalRead(buttonPin);

  int inCalibration = 0;
  while (buttonState == HIGH) { 
    inCalibration = 1;

    Serial.println("CALIBRATION MODE");
    servo.write(servocenter); 

    rssiLeft = analogRead(inputLeft);
    rssiRight = analogRead(inputRight) ;

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

  } 
  if(inCalibration == 1){
    Serial.println("Writing epprom....");
    EEPROM.write(1,offsetLeft);
    EEPROM.write(2,offsetRight);
    inCalibration = 0; 
    delay(2000);
  }  


  int offsetLeft = EEPROM.read(1);
  int offsetRight = EEPROM.read(2);

  //rssiLeftRaw = ;
  //rssiRightRaw = analogRead(inputRight) + offsetRight;
  
  rssiLeft = kalman_rssiLeft_update(analogRead(inputLeft) + offsetLeft);
  rssiRight = kalman_rssiRight_update(analogRead(inputRight) + offsetRight);
  
  rssiMidFront = analogRead(inputMidFront); 
  rssiMidTop = analogRead(inputMidTop); 

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
      delay(lostlockhold);    
      hasSignal = 0;          
    } 
  } 
  else {  
    hasSignal = 1; 

    if(rssiDiff <= smoothing){
      speed = 0;
    } 
    else if (rssiDiff <= smoothing + 1){
      speed = 1 ;
    } 
    else if (rssiDiff <= smoothing + 2){
      speed = 2 ;
    } 
    else if (rssiDiff <= smoothing + 4){
      speed = 3 ;
    } 
    else if (rssiDiff <= smoothing + 6){
      speed = 4 ;
    } 
    else if (rssiDiff <= smoothing + 8){
      speed = 5 ;
    } 
    else if (rssiDiff <= smoothing + 10){
      speed = 6 ;
    } 
    else if (rssiDiff <= smoothing + 12){
      speed = 8 ;
    } 
    else if (rssiDiff <= smoothing + 14){
      speed = 10 ;
    }  
    else if (rssiDiff <= smoothing + 16){
      speed = 12 ;
    }     
    else if (rssiDiff <= smoothing + 18){
      speed = 14 ;
    }else if (rssiDiff <= smoothing + 20){
      speed = 16 ;    
    }else if (rssiDiff <= smoothing + 22){
      speed = 18 ; 
    }else if (rssiDiff <= smoothing + 24){
      speed = 20 ;          
    } else {
      speed = maxspeed; 
    }  


    //track
    if(rssiLeft > rssiRight) { 
      //Serial.println("move left");   
      servo.write(servocenter +  speed);   
    }
    else if(rssiRight > rssiLeft) { 
      //Serial.println("move right");  
      servo.write(servocenter -  speed);
    } 
    else if (rssiDiff < smoothing) {
      //Serial.println("center");  
      servo.write(servocenter);
      delay(centerhold);
    }  
  }     

}









