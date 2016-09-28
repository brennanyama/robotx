int ch1, linear, inv_linear; // linear drive
int ch2, strafe; //turning left joystick
int ch3, rotate, inv_rotate; //rotation
int motor_1 = 12;
int motor_2 = 8;
int motor_3 = 9;
int motor_4 = 10; // Motors (PortForward, StarboardForward, PortAft, StarboardAft)

void setup() {
  
pinMode(5, INPUT); //reading in linear
//pinMode(6, INPUT); //reading in strafe
pinMode(3, INPUT); //reading in rotate
pinMode(12, OUTPUT); //PWM controls to motors
pinMode(8, OUTPUT);
pinMode(9, OUTPUT);
pinMode(10, OUTPUT);
pinMode(2, INPUT_PULLUP);


attachInterrupt(digitalPinToInterrupt(2), Recon, HIGH); //interrupt function called Recon -- called when RX is turned on

Serial.begin(9600);
  
}

void loop() {

  ch1 = pulseIn(5, HIGH, 25000);
//  ch2 = pulseIn(6, HIGH, 25000);
  ch3 = pulseIn(3, HIGH, 25000);
  Serial.println("------------");
  linear = map(ch1, 1130, 1869, 90, 255);
//  strafe = map(ch2, 1130, 1869, 90, 255);
  rotate = map(ch3, 1130, 1960, 90, 255);  // for propeller motor controller 500 < p < 2500
  inv_rotate = map(rotate, 90, 255, 255, 90);
  
    Serial.println(ch1);
//    Serial.println(ch2);
    Serial.println(ch3);
    Serial.println("linear: "); // 3 on RX
    Serial.println(linear);
//    Serial.println("strafe "); //  4 on RX
//    Serial.println(strafe);
    Serial.println("rotate: "); // 1 on the RX
    Serial.println(rotate);

    if(linear <= 190 && linear >= 170 ){
      //define neutral
      analogWrite(motor_1, 190);
      analogWrite(motor_2, 190);
      analogWrite(motor_3, 190);
      analogWrite(motor_4, 190);  
    
       if(rotate < 170 || rotate > 200){
        //define rotation
        
        analogWrite(motor_1, rotate);
        analogWrite(motor_2, inv_rotate);
        analogWrite(motor_3, rotate);
        analogWrite(motor_4, inv_rotate);
      }
    }
    else{
        //define basic forward/reverse movement
        analogWrite(motor_1, linear); //forward or reverse
        analogWrite(motor_2, linear);
        analogWrite(motor_3, linear);
        analogWrite(motor_4, linear);  
      }
    
    
//    if(strafe >= 190){ //strafe right
//      analogWrite(motor_1, linear-strafe);
//      analogWrite(motor_4, linear-strafe);
//    }
//    else if(strafe <= 170){ //strafe left
//      analogWrite(motor_2, linear+strafe);
//      analogWrite(motor_3, linear+strafe);
//    }
//    else if(rotate >= 50){
//    analogWrite(motor_1, -abs(linear));
//    analogWrite(motor_2, abs(linear));
//    analogWrite(motor_3, -abs(linear));
//    analogWrite(motor_4, abs(linear));  
//    }
//    else if(rotate <=-50){
//    analogWrite(motor_1, abs(linear));
//    analogWrite(motor_2, -abs(linear));
//    analogWrite(motor_3, abs(linear));
//    analogWrite(motor_4, -abs(linear));
//    
//    }
//    Serial.println("motor4: ");
//    Serial.println(2);

  delay(1000);
}

void Recon(){
  while(2 == HIGH){ //while the RX is on
    
//  digitalWrite(11, HIGH); //blink
//  delay(500);
//  digitalWrite(11, LOW);
//  delay(500);
  }
}


