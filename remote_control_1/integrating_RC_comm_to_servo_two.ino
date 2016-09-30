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

const int sQ1_pin = 8;
const int sQ2_pin = 9;
const int sQ3_pin = 10;
const int sQ4_pin = 11;


int ch1, linear, inv_linear; // linear drive
int ch2, strafe; //turning left joystick
int ch3, rotate, inv_rotate; //rotation
int map_servoQ1, map_servoQ2, map_servoQ3, map_servoQ4;
// Instantiate node handle globally (tell ROS we're creating publishers and subscribers))
ros::NodeHandle  nh;

// Create servo objects (no ROS stuff here...we're utilizing the Arduino servo library to control servos)
Servo servoQ1;  // initialize servo 8 object (front left / fore port)
Servo servoQ2;  // initialize servo 9 object (front right / fore starboard)
Servo servoQ3;  // initialize servo 10 object (rear right / aft starboard)
Servo servoQ4;  // initialize servo 11 object (front left / aft port)

// Instantiate ROS publisher message
//remote control message
std_msgs::Bool take_over_msg;
std_msgs::UInt16 re_con_msg_lin;
std_msgs::UInt16 re_con_msg_rot;
std_msgs::UInt16 re_con_msg_inv_rot;

// Instantiate ROS subscriber message and callback function
void servoQ1_msg(const std_msgs::UInt16& cmd_msg){
  map_servoQ1 = map(cmd_msg.data, -100, 100, 0, 255);
  servoQ1.write(map_servoQ1);        // set servo angle, should be from 0-180  
}
void servoQ2_msg(const std_msgs::UInt16& cmd_msg){
  map_servoQ2 = map(cmd_msg.data, -100, 100, 0, 255);
  servoQ2.write(map_servoQ2);        // set servo angle, should be from 0-180  
}
void servoQ3_msg(const std_msgs::UInt16& cmd_msg){
  map_servoQ3 = map(cmd_msg.data, -100, 100, 0, 255);
  servoQ3.write(map_servoQ3);        // set servo angle, should be from 0-180  
}
void servoQ4_msg(const std_msgs::UInt16& cmd_msg){
  map_servoQ4 = map(cmd_msg.data, -100, 100, 0, 255);
  servoQ4.write(map_servoQ4);        // set servo angle, should be from 0-180  
}


// Define rosserial objects (publishers and subscribers) globally
//define remote control publisher
ros::Publisher take_over("take_over", &take_over_msg);
ros::Publisher re_con_linear("remote_control_linear", &re_con_msg_lin);
ros::Publisher re_con_rotate("remote_control_linear", &re_con_msg_rot);
ros::Publisher re_con_inv_rotate("remote_control_linear", &re_con_msg_inv_rot);
ros::Subscriber<std_msgs::UInt16> servoQ1_sub("/servo8", servoQ1_msg);
ros::Subscriber<std_msgs::UInt16> servoQ2_sub("/servo9", servoQ2_msg);
ros::Subscriber<std_msgs::UInt16> servoQ3_sub("/servo10", servoQ3_msg);
ros::Subscriber<std_msgs::UInt16> servoQ4_sub("/servo11", servoQ4_msg);

void setup(){
  
  // Instantiate node handle
  nh.initNode();
  
  // Instantiate pubish (advertise) and subscribe node handles
  //remote control publish
  nh.advertise(take_over);
  nh.advertise(re_con_linear);
  nh.advertise(re_con_rotate);
  nh.advertise(re_con_inv_rotate);
  pinMode(4, INPUT); //reading in linear
  pinMode(3, INPUT); //reading in rotate
  nh.subscribe(servoQ1_sub);
  nh.subscribe(servoQ2_sub);
  nh.subscribe(servoQ3_sub);
  nh.subscribe(servoQ4_sub);

  // Attach servo objects to pins
  servoQ1.attach(sQ1_pin);
  servoQ2.attach(sQ2_pin);
  servoQ3.attach(sQ3_pin);
  servoQ4.attach(sQ4_pin); 
}

void loop(){
  ch1 = pulseIn(3, HIGH, 25000);
  ch2 = pulseIn(4, HIGH, 25000);
  linear = map(ch1, 1130, 1869, 0, 180);
  rotate = map(ch2, 1130, 1960, 0, 180);  // for propeller motor controller 500 < p < 2500
  inv_rotate = map(rotate, 0, 180, 180, 0);
  int x= digitalRead(30);
  if(linear >=0 && linear <=180){
    take_over_msg.data = HIGH;
    //remote control data
    re_con_msg_lin.data = linear;
    re_con_linear.publish(&re_con_msg_lin);
    
    if(linear >= 70 && linear <= 90 ){
      //define neutral
      servoQ1.write(0);
      servoQ1.write(0);
      servoQ1.write(0);
      servoQ1.write(0);
       if(rotate < 70 || rotate > 100){
        //define rotation
        servoQ1.write(inv_rotate);
        servoQ2.write(rotate);
        servoQ3.write(rotate);
        servoQ4.write(inv_rotate);
      }
    }
    else{ //define basic forward/reverse movement
        servoQ1.write(linear); //forward or reverse
        servoQ2.write(linear);
        servoQ3.write(linear);
        servoQ4.write(linear);  
      }
  }
  else{
    take_over_msg.data = LOW;
  }
  take_over.publish( &take_over_msg);
  
  
  nh.spinOnce();
  delay(10);
}

