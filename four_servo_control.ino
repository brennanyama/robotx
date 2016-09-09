/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

// Instantiate node handle (tell ROS we're creating publishers and subscribers))
ros::NodeHandle  nh;

// Create servo objects (no ROS stuff here...we're utilizing the Arduino servo library to control servos)
Servo servo8;   // initialize servo 8 object (front left / fore port)
Servo servo9;   // initialize servo 9 object (front right / fore starboard)
Servo servo10;  // initialize servo 10 object (rear right / aft starboard)
Servo servo11;  // initialize servo 11 object (front left / aft port)

// Instantiate ROS subscriber callback function
void servo8_cb(const std_msgs::UInt16& cmd_msg){
  servo8.write(cmd_msg.data);        // set servo angle, should be from 0-180  
}
void servo9_cb(const std_msgs::UInt16& cmd_msg){
  servo9.write(cmd_msg.data);        // set servo angle, should be from 0-180  
}
void servo10_cb(const std_msgs::UInt16& cmd_msg){
  servo10.write(cmd_msg.data);        // set servo angle, should be from 0-180  
}
void servo11_cb(const std_msgs::UInt16& cmd_msg){
  servo11.write(cmd_msg.data);        // set servo angle, should be from 0-180  
}

// Instantiate the publishers and subscribers
ros::Subscriber<std_msgs::UInt16> servo8_sub("servo8", servo8_cb);
ros::Subscriber<std_msgs::UInt16> servo9_sub("servo9", servo9_cb);
ros::Subscriber<std_msgs::UInt16> servo10_sub("servo10", servo10_cb);
ros::Subscriber<std_msgs::UInt16> servo11_sub("servo11", servo11_cb);

void setup(){
  nh.initNode();
  nh.subscribe(servo8_sub);     // subscribe to servo8 topic
  nh.subscribe(servo9_sub);     // subscribe to servo8 topic
  nh.subscribe(servo10_sub);    // subscribe to servo8 topic
  nh.subscribe(servo11_sub);    // subscribe to servo8 topic
  servo8.attach(8);             // attach servo8 object to pin 8
  servo9.attach(9);             // attach servo9 object to pin 9
  servo10.attach(10);           // attach servo10 object to pin 10
  servo11.attach(11);           // attach servo11 object to pin 11
}

void loop(){
  nh.spinOnce();
  delay(1);
}
