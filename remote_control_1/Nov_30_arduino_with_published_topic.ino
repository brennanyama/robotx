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
int count = 0;

std_msgs::UInt16 test;
std_msgs::UInt16 test2;
std_msgs::UInt16 test3;
std_msgs::UInt16 test4;
std_msgs::UInt16 wamvstatus;
ros::NodeHandle nh;

void q1_cb(const std_msgs::UInt16& cmd_msg){
  auto_q1= map(cmd_msg.data, 0, 100, 100, 255);
  analogWrite(8, auto_q1);
}
void q2_cb(const std_msgs::UInt16& cmd_msg){
  auto_q2= map(cmd_msg.data, 0, 100, 100, 255);
  analogWrite(9, auto_q2);
}
void q3_cb(const std_msgs::UInt16& cmd_msg){
  auto_q3= map(cmd_msg.data, 0, 100, 100, 255);
  analogWrite(10, auto_q3);
}
void q4_cb(const std_msgs::UInt16& cmd_msg){
  auto_q4= map(cmd_msg.data, 0, 100, 100, 255);
  analogWrite(11, auto_q4);
}

ros::Subscriber<std_msgs::UInt16> motor_q1_sub("motor_q1", q1_cb);
ros::Subscriber<std_msgs::UInt16> motor_q2_sub("motor_q2", q2_cb);
ros::Subscriber<std_msgs::UInt16> motor_q3_sub("motor_q3", q3_cb);
ros::Subscriber<std_msgs::UInt16> motor_q4_sub("motor_q4", q4_cb);
ros::Publisher q1_pub("q1_pub", &test);
ros::Publisher q2_pub("q2_pub", &test2);
ros::Publisher q3_pub("q3_pub", &test3);
ros::Publisher q4_pub("q4_pub", &test4);
ros::Publisher status_pub("status_pub", &wamvstatus);

void setup(){
  nh.initNode();
  nh.subscribe(motor_q1_sub);
  nh.subscribe(motor_q2_sub);
  nh.subscribe(motor_q3_sub);
  nh.subscribe(motor_q4_sub);
  nh.advertise(q1_pub);
  nh.advertise(q2_pub);
  nh.advertise(q3_pub);
  nh.advertise(q4_pub); 
  nh.advertise(status_pub); 

  
  pinMode(4, INPUT); //reading in linear
  pinMode(3, INPUT); //reading in rotate
  pinMode(8, OUTPUT);   //Q1
  pinMode(9, OUTPUT);   //Q2
  pinMode(10, OUTPUT);  //Q3
  pinMode(11, OUTPUT);  //Q4
  pinMode(48, OUTPUT); //red
  pinMode(42, OUTPUT); //yellow
  pinMode(38, OUTPUT); //green
  pinMode(36, INPUT); // kill switch
  
}

void loop(){
  ch1 = pulseIn(3, HIGH, 25000);
  ch2 = pulseIn(4, HIGH, 25000);
  rotate = map(ch2, 1130, 1960, 100, 245);
  linear = map(ch1, 1130, 1869, 245, 100);
  inv_rotate = map(rotate, 100, 245, 245, 100);
  back_linear = map(linear, 245, 100, 100, 245); 
  
  if(digitalRead(36) == HIGH){
    wamvstatus.data = 3;
    digitalWrite(42, LOW);
    digitalWrite(48, LOW);
    digitalWrite(38, HIGH);
  }
  else if(linear > 50 && linear < 244){
    noInterrupts();
    wamvstatus.data = 2;
    digitalWrite(42, LOW);
    digitalWrite(48, HIGH);
    digitalWrite(40, LOW);
    analogWrite(8, linear);
    analogWrite(9, linear);
    analogWrite(10, linear);
    analogWrite(11, linear);

    if(rotate> 180){ //rotate right
      analogWrite(8, rotate);
      analogWrite(9, inv_rotate);
      analogWrite(10, rotate);
      analogWrite(11, inv_rotate);   
    }
    
    if(rotate<140){ //rotate left
      analogWrite(8, rotate);
      analogWrite(9, inv_rotate);
      analogWrite(10, rotate);
      analogWrite(11, inv_rotate);
    }
    
  }
  else{
    wamvstatus.data = 1;
    interrupts();
    digitalWrite(42, HIGH);
    digitalWrite(48, LOW);
    digitalWrite(40, LOW);
  }
//publish
  status_pub.publish(&wamvstatus);
  test.data = analogRead(8);
  test2.data = analogRead(9);
  test3.data = analogRead(10);
  test4.data = analogRead(11);
  q1_pub.publish(&test);
  q2_pub.publish(&test2);
  q3_pub.publish(&test3);
  q4_pub.publish(&test4);
  
  nh.spinOnce();
  delay(1);
  
}



