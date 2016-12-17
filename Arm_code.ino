/* Libraries*/
#include <Stepper.h>
#include <Servo.h>
#include <math.h>

//#include <MatrixMath.h>
#include <ros.h>
#include <std_msgs/Int8.h>

#define encoderPinA 2          
#define encoderPinB 3
#define motor 11                   
#define limitswitch 10


/*Variables*/
//Stepper
int L = 138;
const int stepsperrev = 200;      
int in1Pin = 4;           
int in2Pin = 5;           
int in3Pin = 6;           
int in4Pin = 7;


//DC motor
volatile int encoderPos = 1;    
int A = digitalRead(encoderPinA);
int B = digitalRead(encoderPinB);
int istate;
int laststate;             
//int LS;
double lastpos = 1;
int desangle;
int stepf = 0;
int steps = 0;
int start = 0;
int reset = 0;
int started;
int stoparm;

Servo myservo;       
Stepper myStepper(200, in1Pin, in2Pin, in3Pin, in4Pin);

//double xstart,ystart;
//double xfinish,yfinish;
//double desanglefinish,theta2finish;
//double xdelta,ydelta;
//double desangle,theta2;
//double desangledelta,Ddelta;
//double Jac[2][2];          
//double D;

ros::NodeHandle nh;

void rosreset(const std_msgs::Int8& cmd_msg)
{
  reset = cmd_msg.data;
  
}

void rosstop(const std_msgs::Int8& cmd_msg)
{
  armstop = cmd_msg.data;
  
}

void rosangle(const std_msgs::Int8& cmd_msg)
{
  desangle = cmd_msg.data;
  
}

void rosstart(const std_msgs::Int8& cmd_msg)
{
  start = cmd_msg.data;
  
}

void rosextension(const std_msgs::Int8& cmd_msg)
{
  steps = cmd_msg.data*454.545455;
}
  
void doEncoderA()             //when doencoder is called, increase position
{
  encoderPos++;
}

void doEncoderB()             //when doencoder is called, increase position
{
  encoderPos++;
}


ros::Subscriber<std_msgs::Int8> stop_sub("armstop", rosstop);
ros::Subscriber<std_msgs::Int8> reset_sub("armreset", rosreset);
ros::Subscriber<std_msgs::Int8> start_sub("armstart", rosstart);
ros::Subscriber<std_msgs::Int8> angle_sub("armangle", rosangle);
ros::Subscriber<std_msgs::Int8> extension_sub("armextension", rosextension);

void setup() {
  nh.initNode();
  nh.subscribe(angle_sub);
  nh.subscribe(extension_sub);
  nh.subscribe(start_sub);
  nh.subscribe(reset_sub);
  // Stepper motor
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  //DC motor
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(motor, OUTPUT);
  pinMode(limitswitch, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, RISING);    
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, RISING);
  Serial.begin (9600);
  Serial.println("Setup successful...");
  myStepper.setSpeed(200);
  istate = A;

  myservo.attach (9);       
  
                          

}

void loop() {
  if(start == 1 )
  {
    started = 1;
    encoderPos = 0;

    myservo.write (150);

      nh.spinOnce();
      delay(1);
    analogWrite(motor, 220); 

//    LS = digitalRead(limitswitch);
//    if (LS == LOW){
//      Serial.println("ktk");
//      Serial.println(lastpos);
//      Serial.println(desangle);
      
//      delay(100);
//    }
//    if (LS == HIGH){ 
//      analogWrite(motor, 180);
//      encoderPos = 0;
//      Serial.println ("LS Pressed");
//      delay(100);
//    }
    
      if (encoderPos == 0)
      {
        while (((desangle/.1536)*3.15) > encoderPos)
        {                       //desiredPos*3.19 converts angles into counts
          
          Serial.println("Ch A");                                         // see state of ch A
          Serial.println(A);
          Serial.println("Ch B");                                         // see state of ch B       
          Serial.println(B);  
          Serial.println("Encoder Pos");  
          Serial.println(encoderPos);  
//  Serial.println("theta2");  
//  Serial.println(theta2);  
          Serial.println("lastpos");
          Serial.println(lastpos);                                          
          analogWrite(motor, 200);                                      //moves arm down
          laststate = digitalRead(encoderPinA);

          if (istate != laststate)
          {                                     //if the motor is moving, it will try to change encoderposition accordingly

            if (A != B)
            {
              Serial.println("clockwise");
              encoderPos++;
            }
            if (B > A)
            { 
              Serial.println("counterclockwise");
              encoderPos++;
            }
    
            Serial.println(encoderPos);
            Serial.println("-----------------------------");            //for neatness sake, seperate loops to easily identify changes
          }
          lastpos = desangle;
        }
      delay(10);
      analogWrite(motor, 180);

      nh.spinOnce();
      delay(1);
      
      Serial.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
      myStepper.step((-steps*454.545455));
      delay (2000);
      stepf = steps;
      myservo.write (150);          // sets the servo at 0 degree position
      delay(2000);                // waits for the servo to get there
      myservo.write (90);         // sets the servo at 90 degree position
      delay (4000);               // waits for the servo to get there
      myservo.write (150);
      delay (1500);
//    myStepper.step(steps*454.545455);
//    delay (2000);

      

      
     }
    }


  
  while (reset != 1 && started == 1)
  {
  nh.spinOnce();
  delay(1);
    if( reset == 1)
    {
      myStepper.step((stepf*454.545455));
      delay (2000);
      started = 0;
      while ( encoderPos > 100)
        {                       //desiredPos*3.19 converts angles into counts
          if( armstop == 1)
          {
            encoderPos = 0;
          }
          Serial.println("Ch A");                                         // see state of ch A
          Serial.println(A);
          Serial.println("Ch B");                                         // see state of ch B       
          Serial.println(B);  
          Serial.println("Encoder Pos");  
          Serial.println(encoderPos);  
//  Serial.println("theta2");  
//  Serial.println(theta2);  
          Serial.println("lastpos");
          Serial.println(lastpos);                                          
          analogWrite(motor, 130);                                      //moves arm down
          laststate = digitalRead(encoderPinA);

          if (istate != laststate)
          {                                     //if the motor is moving, it will try to change encoderposition accordingly

            if (A != B)
            {
              Serial.println("clockwise");
              encoderPos--;
            }
            if (A > B)
            { 
              Serial.println("counterclockwise");
              encoderPos--;
            }
    
            Serial.println(encoderPos);
            Serial.println("-----------------------------");            //for neatness sake, seperate loops to easily identify changes
          }
          lastpos = desangle;
        }
    }
  }

    nh.spinOnce();
    delay(1);
        
}
