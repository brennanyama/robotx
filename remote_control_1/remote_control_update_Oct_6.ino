/*
 * Firmware for a holonomic wheeled robot with 8 bump sensors
 * 
 * Parts of this code used from ros tutorial found here, 
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>

int ch1, linear; // linear drive
int ch2; //turning left joystic
int rotate, inv_rotate; //rotation


void setup(){
  Serial.begin(9600);
  pinMode(4, INPUT); //reading in linear
  pinMode(3, INPUT); //reading in rotate
  pinMode(8, OUTPUT);   //Q1
  pinMode(9, OUTPUT);   //Q2
  pinMode(10, OUTPUT);  //Q3
  pinMode(11, OUTPUT);  //Q4
  
}

void loop(){
  ch1 = pulseIn(3, HIGH, 25000);
  ch2 = pulseIn(4, HIGH, 25000);
  rotate = map(ch2, 1130, 1960, 100, 245);
  linear = map(ch1, 1130, 1869, 255, 100);
  inv_rotate = map(rotate, 100, 245, 245, 100);
  Serial.println(rotate);
  
  if(linear > 100 && linear < 255){
    analogWrite(8, linear);
    analogWrite(9, linear);
    analogWrite(10, linear);
    analogWrite(11, linear);

    if(rotate> 210){ //rotate right
      analogWrite(8, inv_rotate);
      analogWrite(9, rotate);
      analogWrite(10, rotate);
      analogWrite(11, inv_rotate);   
    }
    
    if(rotate<150){ //rotate left
      analogWrite(8, rotate);
      analogWrite(9, inv_rotate);
      analogWrite(10, inv_rotate);
      analogWrite(11, rotate);
    }
  }
}

