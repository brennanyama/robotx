/* Libraries*/
#include <Stepper.h>
#include <Servo.h>
#include <math.h>
#include <MatrixMath.h>

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
int LS;
double lastpos = 0;
int desangle = 30;
int steps = 20;
int waka = 0;

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

void doEncoderA()             //when doencoder is called, increase position
{
  encoderPos++;
}

void doEncoderB()             //when doencoder is called, increase position
{
  encoderPos++;
}

void setup() {

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
    //Serial.println("Type In 2 Numbers for Target");
    //while (Serial.available() == 0);
     //desangle = Serial.parseInt();
     //steps = Serial.parseInt();

    //Serial.print("numbers are ");
    //Serial.print(xfinish); Serial.print("cm");
    //Serial.print(yfinish); Serial.print("cm");
    
//    double xdelta = (xfinish-xstart);                    
//    double ydelta = (yfinish-ystart);
    myservo.write (150);
    delay(100);
    while ((desangle != lastpos)) {
    
    analogWrite(motor, 170);
    LS = digitalRead(limitswitch);
    if (LS == LOW){
      Serial.println("ktk");
//      Serial.println(theta2);
      Serial.println(lastpos);
      
      delay(100);
    }
    if (LS == HIGH){ 
      analogWrite(motor, 180);
      encoderPos = 0;
      Serial.println ("LS Pressed");
      delay(100);
    }
    
  if (encoderPos == 0){
  while (((desangle/.1536)*3.15) > encoderPos){                       //desiredPos*3.19 converts angles into counts
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
   
    if (istate != laststate){                                     //if the motor is moving, it will try to change encoderposition accordingly

    if (A != B){
      Serial.println("clockwise");
      encoderPos++;
    }
    if (B > A){ 
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

    if (steps == 10){
    Serial.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
    myStepper.step((-steps*454.545455));
    delay (2000);

    myservo.write (150);          // sets the servo at 0 degree position
    delay(2000);                // waits for the servo to get there
    myservo.write (90);         // sets the servo at 90 degree position
    delay (4000);               // waits for the servo to get there
    myservo.write (150);
    delay (1500);
    myStepper.step(steps*454.545455);
    delay (2000);

    encoderPos = 1;
    waka = 1;
      }
      while (waka == 1){
    LS = digitalRead(limitswitch);  
    if (LS == LOW){  
      analogWrite (motor, 170);
      Serial.println("ktk");
      delay(10);
    }
    if (LS == HIGH){ 
      analogWrite(motor, 180);
      Serial.println ("LS Pressed");
      delay(10);
      waka = 0;
     
    }
       
      }
      
     }
    }
}


