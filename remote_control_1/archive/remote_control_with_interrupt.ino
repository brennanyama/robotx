
//int throttlePin = 4;
int ch1, linear; // linear drive
int ch2, strafe; //turning left joystick
int ch3, rotate; //rotation
int motor_1 = 7;
int motor_2 = 8;
int motor_3 = 9;
int motor_4 = 10; // Motors (PortForward, StarboardForward, PortAft, StarboardAft)


void setup() {
  
pinMode(3, INPUT); //reading in linear
pinMode(4, INPUT); //reading in strafe
pinMode(5, INPUT); //reading in rotate
pinMode(11, OUTPUT); //LED to test interrupt_loop
pinMode(7, OUTPUT); //PWM controls to motors
pinMode(8, OUTPUT);
pinMode(9, OUTPUT);
pinMode(10, OUTPUT);
attachInterrupt(18, Recon, HIGH); //interrupt function called Recon -- called when RX is turned on

Serial.begin(9600);
  
}

void loop() {
  digitalWrite(11, HIGH); //blink
  delay(500);
  digitalWrite(11, LOW);
  delay(500);
}

void Recon(){
  while(18 == 1){ //while the RX is on
  ch1 = pulseIn(3, HIGH, 25000);
  ch2 = pulseIn(4, HIGH, 25000);
  ch3 = pulseIn(5, HIGH, 25000);
  
  linear = map(ch1, 1000, 2000, -255, 255);
  strafe = map(ch2, 1000, 2000, -255, 255);
  rotate = map(ch3, 1000, 2000, -255, 255);
   
    analogWrite(motor_1, linear); //forward or reverse
    analogWrite(motor_2, linear);
    analogWrite(motor_3, linear);
    analogWrite(motor_4, linear);
    
    if(strafe >= 50){ //strafe right
      analogWrite(motor_1, linear-strafe);
      analogWrite(motor_4, linear-strafe);
    }
    else if(strafe <=-50){ //strafe left
      analogWrite(motor_2, linear+strafe);
      analogWrite(motor_3, linear+strafe);
    }
    else if(rotate >= 50){
    analogWrite(motor_1, -abs(linear));
    analogWrite(motor_2, abs(linear));
    analogWrite(motor_3, -abs(linear));
    analogWrite(motor_4, abs(linear));  
    }
    else if(rotate <=-50){
    analogWrite(motor_1, abs(linear));
    analogWrite(motor_2, -abs(linear));
    analogWrite(motor_3, abs(linear));
    analogWrite(motor_4, -abs(linear));
    }
  }
}


