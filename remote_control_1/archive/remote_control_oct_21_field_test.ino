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

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
int ch1, linear; // linear drive
int ch2; //turning left joystic
int rotate, inv_rotate; //rotation
int auto_q1, auto_q2, auto_q3, auto_q4;
int back_linear;
float multiplier = 1 - 13/16.6;
std_msgs::UInt16 test;
ros::NodeHandle nh;

void q1_cb(const std_msgs::UInt16& cmd_msg){
  auto_q1= map(cmd_msg.data, 0, 100, 100, 245);
  analogWrite(8, multiplier*auto_q1);
}
void q2_cb(const std_msgs::UInt16& cmd_msg){
  auto_q2= map(cmd_msg.data, 0, 100, 100, 245);
  analogWrite(9, multiplier*auto_q2);
}
void q3_cb(const std_msgs::UInt16& cmd_msg){
  auto_q3= map(cmd_msg.data, 0, 100, 100, 245);
  analogWrite(10, auto_q3);
}
void q4_cb(const std_msgs::UInt16& cmd_msg){
  auto_q4= map(cmd_msg.data, 0, 100, 100, 245);
  analogWrite(11, auto_q4);
}

ros::Subscriber<std_msgs::UInt16> motor_q1_sub("motor_q1", q1_cb);
ros::Subscriber<std_msgs::UInt16> motor_q2_sub("motor_q2", q2_cb);
ros::Subscriber<std_msgs::UInt16> motor_q3_sub("motor_q3", q3_cb);
ros::Subscriber<std_msgs::UInt16> motor_q4_sub("motor_q4", q4_cb);
ros::Publisher q1_pub("linear", &test);

void setup(){
  nh.initNode();
  nh.subscribe(motor_q1_sub);
  nh.subscribe(motor_q2_sub);
  nh.subscribe(motor_q3_sub);
  nh.subscribe(motor_q4_sub);
  nh.advertise(q1_pub);
  
  pinMode(4, INPUT); //reading in linear
  pinMode(3, INPUT); //reading in rotate
  pinMode(8, OUTPUT);   //Q1
  pinMode(9, OUTPUT);   //Q2
  pinMode(10, OUTPUT);  //Q3
  pinMode(11, OUTPUT);  //Q4
  
}

void loop(){
  int count = 0;
//  int sensorValue = analogRead(A0);
//  float voltage = sensorValue * (5.0/ 1023.0);
//  float Vin = map(voltage, 0,4.6,0,16);
//  float adjustment = 3.4/voltage;
//  float multiply;
//  if(adjustment >1){
//      multiply = 1;
//  }
//  else{ 
//    multiply = adjustment + .03;
//  }
  
  ch1 = pulseIn(3, HIGH, 25000);
  ch2 = pulseIn(4, HIGH, 25000);
  rotate = map(ch2, 1130, 1960, 100, 245);
  linear = map(ch1, 1130, 1869, 255, 100);
  inv_rotate = map(rotate, 100, 245, 245, 100);
  back_linear = map(ch1, 1130, 1960, 240, 130); 
  
  if(linear > 100 && linear < 255){
    analogWrite(8, multiplier*back_linear);
    analogWrite(9, multiplier*back_linear);
    analogWrite(10, linear);
    analogWrite(11, linear);

    if(rotate> 210){ //rotate right
      analogWrite(8, multiplier*inv_rotate);
      analogWrite(9, multiplier*rotate);
      analogWrite(10, rotate);
      analogWrite(11, inv_rotate);   
    }
    
    if(rotate<150){ //rotate left
      analogWrite(8, multiplier*rotate);
      analogWrite(9, multiplier*inv_rotate);
      analogWrite(10, inv_rotate);
      analogWrite(11, rotate);
    }
    
  }
  
  if(count == 100){
    test.data = linear;
    q1_pub.publish(&test);
    count = 0;
  }
  count ++;
  nh.spinOnce();
  delay(1);
  
}

