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

// Instantiate node handle globally (tell ROS we're creating publishers and subscribers))
ros::NodeHandle  nh;

// Create servo objects (no ROS stuff here...we're utilizing the Arduino servo library to control servos)
Servo servoQ1;  // initialize servo 8 object (front left / fore port)
Servo servoQ2;  // initialize servo 9 object (front right / fore starboard)
Servo servoQ3;  // initialize servo 10 object (rear right / aft starboard)
Servo servoQ4;  // initialize servo 11 object (front left / aft port)

// Instantiate ROS publisher message
std_msgs::Bool bump22_msg;
std_msgs::Bool bump24_msg;
std_msgs::Bool bump26_msg;
std_msgs::Bool bump28_msg;
std_msgs::Bool bump30_msg;
std_msgs::Bool bump32_msg;
std_msgs::Bool bump34_msg;
std_msgs::Bool bump36_msg;

// Instantiate ROS subscriber message and callback function
void servoQ1_msg(const std_msgs::UInt16& cmd_msg){
  servoQ1.write(cmd_msg.data);        // set servo angle, should be from 0-180  
}
void servoQ2_msg(const std_msgs::UInt16& cmd_msg){
  servoQ2.write(cmd_msg.data);        // set servo angle, should be from 0-180  
}
void servoQ3_msg(const std_msgs::UInt16& cmd_msg){
  servoQ3.write(cmd_msg.data);        // set servo angle, should be from 0-180  
}
void servoQ4_msg(const std_msgs::UInt16& cmd_msg){
  servoQ4.write(cmd_msg.data);        // set servo angle, should be from 0-180  
}

// Define rosserial objects (publishers and subscribers) globally
ros::Publisher bump22_sub("bump22", &bump22_msg);
ros::Publisher bump24_sub("bump24", &bump24_msg);
ros::Publisher bump26_sub("bump26", &bump26_msg);
ros::Publisher bump28_sub("bump28", &bump28_msg);
ros::Publisher bump30_sub("bump30", &bump30_msg);
ros::Publisher bump32_sub("bump32", &bump32_msg);
ros::Publisher bump34_sub("bump34", &bump34_msg);
ros::Publisher bump36_sub("bump36", &bump36_msg);
ros::Subscriber<std_msgs::UInt16> servoQ1_sub("/servo8", servoQ1_msg);
ros::Subscriber<std_msgs::UInt16> servoQ2_sub("/servo9", servoQ2_msg);
ros::Subscriber<std_msgs::UInt16> servoQ3_sub("/servo10", servoQ3_msg);
ros::Subscriber<std_msgs::UInt16> servoQ4_sub("/servo11", servoQ4_msg);

const int bump22_pin = 22;
const int bump24_pin = 24;
const int bump26_pin = 26;
const int bump28_pin = 28;
const int bump30_pin = 30;
const int bump32_pin = 32;
const int bump34_pin = 34;
const int bump36_pin = 36;

bool last_reading22;
bool last_reading24;
bool last_reading26;
bool last_reading28;
bool last_reading30;
bool last_reading32;
bool last_reading34;
bool last_reading36;

long last_debounce_time22 = 0;
long last_debounce_time24 = 0;
long last_debounce_time26 = 0;
long last_debounce_time28 = 0;
long last_debounce_time30 = 0;
long last_debounce_time32 = 0;
long last_debounce_time34 = 0;
long last_debounce_time36 = 0;

bool published22 = true;
bool published24 = true;
bool published26 = true;
bool published28 = true;
bool published30 = true;
bool published32 = true;
bool published34 = true;
bool published36 = true;

long debounce_delay = 50;

void setup(){
  
  // Instantiate node handle
  nh.initNode();
  
  // Instantiate pubish (advertise) and subscribe node handles
  nh.advertise(bump22_sub);
  nh.advertise(bump24_sub);
  nh.advertise(bump26_sub);
  nh.advertise(bump28_sub);
  nh.advertise(bump30_sub);
  nh.advertise(bump32_sub);
  nh.advertise(bump34_sub);
  nh.advertise(bump36_sub);
  nh.subscribe(servoQ1_sub);
  nh.subscribe(servoQ2_sub);
  nh.subscribe(servoQ3_sub);
  nh.subscribe(servoQ4_sub);

  // Set bump sensor pin as input
  pinMode(bump22_pin,INPUT);
  pinMode(bump24_pin,INPUT);
  pinMode(bump26_pin,INPUT);
  pinMode(bump28_pin,INPUT);
  pinMode(bump30_pin,INPUT);
  pinMode(bump32_pin,INPUT);
  pinMode(bump34_pin,INPUT);
  pinMode(bump36_pin,INPUT);

  // Pull bump sensor pin high
  digitalWrite(bump22_pin,HIGH);
  digitalWrite(bump24_pin,HIGH);
  digitalWrite(bump26_pin,HIGH);
  digitalWrite(bump28_pin,HIGH);
  digitalWrite(bump30_pin,HIGH);
  digitalWrite(bump32_pin,HIGH);
  digitalWrite(bump34_pin,HIGH);
  digitalWrite(bump36_pin,HIGH);

  // Decides between normally closed and normally open
  last_reading22 =! digitalRead(bump22_pin);
  last_reading24 =! digitalRead(bump24_pin);
  last_reading26 =! digitalRead(bump26_pin);
  last_reading28 =! digitalRead(bump28_pin);
  last_reading30 =! digitalRead(bump30_pin);
  last_reading32 =! digitalRead(bump32_pin);
  last_reading34 =! digitalRead(bump34_pin);
  last_reading36 =! digitalRead(bump36_pin);
  
  // Attach servo objects to pins
  servoQ1.attach(sQ1_pin);
  servoQ2.attach(sQ2_pin);
  servoQ3.attach(sQ3_pin);
  servoQ4.attach(sQ4_pin); 
}

void loop(){

  bool reading22 =! digitalRead(bump22_pin);
  bool reading24 =! digitalRead(bump24_pin);
  bool reading26 =! digitalRead(bump26_pin);
  bool reading28 =! digitalRead(bump28_pin);
  bool reading30 =! digitalRead(bump30_pin);
  bool reading32 =! digitalRead(bump32_pin);
  bool reading34 =! digitalRead(bump34_pin);
  bool reading36 =! digitalRead(bump36_pin);

  if (last_reading22 != reading22){
    last_debounce_time22 = millis();
    published22 = false;
  }
  if (last_reading24 != reading24){
    last_debounce_time24 = millis();
    published24 = false;
  }
  if (last_reading26 != reading26){
    last_debounce_time26 = millis();
    published26 = false;
  }
  if (last_reading28 != reading28){
    last_debounce_time28 = millis();
    published28 = false;
  }
  if (last_reading30 != reading30){
    last_debounce_time30 = millis();
    published30 = false;
  }
  if (last_reading32 != reading32){
    last_debounce_time32 = millis();
    published32 = false;
  }
  if (last_reading34 != reading34){
    last_debounce_time34 = millis();
    published34 = false;
  }
  if (last_reading36 != reading36){
    last_debounce_time36 = millis();
    published36 = false;
  }

  if (!published22 && (millis() - last_debounce_time22) > debounce_delay){
    bump22_msg.data = reading22;
    bump22_sub.publish(&bump22_msg);
    published22 = true;
  }
  last_reading22 = reading22;
  if (!published24 && (millis() - last_debounce_time24) > debounce_delay){
    bump24_msg.data = reading24;
    bump24_sub.publish(&bump24_msg);
    published24 = true;
  }
  last_reading24 = reading24;
  if (!published26 && (millis() - last_debounce_time26) > debounce_delay){
    bump26_msg.data = reading26;
    bump26_sub.publish(&bump26_msg);
    published26 = true;
  }
  last_reading26 = reading26;
  if (!published28 && (millis() - last_debounce_time28) > debounce_delay){
    bump28_msg.data = reading28;
    bump28_sub.publish(&bump28_msg);
    published28 = true;
  }
  last_reading28 = reading28;
  if (!published30 && (millis() - last_debounce_time30) > debounce_delay){
    bump30_msg.data = reading30;
    bump30_sub.publish(&bump30_msg);
    published30 = true;
  }
  last_reading30 = reading30;
  if (!published32 && (millis() - last_debounce_time32) > debounce_delay){
    bump32_msg.data = reading32;
    bump32_sub.publish(&bump32_msg);
    published32 = true;
  }
  last_reading32 = reading32;
  if (!published34 && (millis() - last_debounce_time34) > debounce_delay){
    bump34_msg.data = reading34;
    bump34_sub.publish(&bump34_msg);
    published34 = true;
  }
  last_reading34 = reading34;
  if (!published36 && (millis() - last_debounce_time36) > debounce_delay){
    bump36_msg.data = reading36;
    bump36_sub.publish(&bump36_msg);
    published36 = true;
  }
  last_reading36 = reading36;
  
  nh.spinOnce();
//  delay(1);
}
