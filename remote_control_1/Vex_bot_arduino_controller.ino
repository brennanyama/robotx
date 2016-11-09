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
int modify, mod_linear;
int count = 0;

std_msgs::UInt16 test;

ros::NodeHandle nh;

void q1_cb(const std_msgs::UInt16& cmd_msg){
  auto_q1= map(cmd_msg.data, 0, 100, 245, 100);
  analogWrite(8, auto_q1);
}
void q2_cb(const std_msgs::UInt16& cmd_msg){
  auto_q2= map(cmd_msg.data, 0, 100, 245, 100);
  analogWrite(9, auto_q2);
}
void q3_cb(const std_msgs::UInt16& cmd_msg){
  auto_q3= map(cmd_msg.data, 0, 100, 245, 120);
  analogWrite(10, auto_q3);
}
void q4_cb(const std_msgs::UInt16& cmd_msg){
  auto_q4= map(cmd_msg.data, 0, 100, 255, 120);
  analogWrite(11, auto_q4);
}

ros::Subscriber<std_msgs::UInt16> motor_q1_sub("motor_q1", q1_cb);
ros::Subscriber<std_msgs::UInt16> motor_q2_sub("motor_q2", q2_cb);
ros::Subscriber<std_msgs::UInt16> motor_q3_sub("motor_q3", q3_cb);
ros::Subscriber<std_msgs::UInt16> motor_q4_sub("motor_q4", q4_cb);
ros::Publisher q1_pub("linear", &test);

void setup() {
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

void loop() {
  
  ch1 = pulseIn(3, HIGH, 25000);
  ch2 = pulseIn(4, HIGH, 25000);
  rotate = map(ch2, 1130, 1960, 120, 250);
  linear = map(ch1, 1130, 1869, 255, 120);
  inv_rotate = map(rotate, 120, 255, 250, 120);
  modify = map(ch2, 1130, 1960, 245 , 130);
  mod_linear = map(ch2, 1130, 1960, 130, 245);
  
  if(linear > 100 && linear < 255){
    noInterrupts();
    analogWrite(8, linear);
    analogWrite(9, linear);
    analogWrite(10, linear);
    analogWrite(11, linear);

    if(rotate> 200){ //rotate right
      analogWrite(8, modify);
      analogWrite(9, mod_linear);
      analogWrite(10, inv_rotate);
      analogWrite(11, rotate);   
    }
    
    if(rotate<170){ //rotate left
      analogWrite(8, modify);
      analogWrite(9, mod_linear);
      analogWrite(10, inv_rotate);
      analogWrite(11, rotate);
    }
    
  }
  else{
    interrupts();
  }

  if(count == 10){
    test.data = linear;
    q1_pub.publish(&test);
    count = 0;
  }
  count ++;
nh.spinOnce();
delay(1);

}
