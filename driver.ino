void setupMotors() {
  pinMode(pinMotorRight_forward, OUTPUT);
  pinMode(pinMotorRight_reverse, OUTPUT);
  pinMode(pinMotorLeft_forward, OUTPUT);
  pinMode(pinMotorLeft_reverse, OUTPUT);
  DPRINTLN("Initial Driving Motor Pins");
}

void stopMotors() {
  analogWrite(pinMotorRight_forward, LOW);   //Motor 1
  analogWrite(pinMotorRight_reverse, LOW);
  analogWrite(pinMotorLeft_forward, LOW);    //Motor 2
  analogWrite(pinMotorLeft_reverse, LOW);
  DPRINTLN("Stop Motors");
  //mower_state = STOP;
}

void moveForward(uint8_t pwmRvalue, uint8_t pwmLvalue) {
  analogWrite(pinMotorRight_reverse, LOW);
  analogWrite(pinMotorRight_forward, pwmRvalue);   //Motor 1
  analogWrite(pinMotorLeft_reverse, LOW);
  analogWrite(pinMotorLeft_forward, pwmLvalue);    //Motor 2
  DPRINTLN("Move Forward");
  //mower_state = GO_FORWARD;
}

void moveBackward(uint8_t pwmRvalue, uint8_t pwmLvalue) {
  analogWrite(pinMotorRight_forward, LOW);       //Motor 1
  analogWrite(pinMotorRight_reverse, pwmRvalue);
  analogWrite(pinMotorLeft_forward, LOW);        //Motor 2
  analogWrite(pinMotorLeft_reverse, pwmLvalue);
  DPRINTLN("Move Backward");
  //mower_state = GO_BACKWARD;
}

void moveLeft(uint8_t pwmValue) {
  analogWrite(pinMotorRight_reverse, LOW);
  analogWrite(pinMotorRight_forward, pwmValue);  //Right wheel forward.
  analogWrite(pinMotorLeft_forward, LOW);
  analogWrite(pinMotorLeft_reverse, pwmValue);   //Left wheel backward.
  DPRINTLN("Move LEFT");
  //mower_state = TURN_LEFT_30;
}

void moveRight(uint8_t pwmValue) {
  analogWrite(pinMotorRight_forward, LOW);
  analogWrite(pinMotorRight_reverse, pwmValue);
  analogWrite(pinMotorLeft_reverse, LOW);
  analogWrite(pinMotorLeft_forward, pwmValue);
  DPRINTLN("Move RIGHT");
  //mower_state = TURN_RIGHT_30;
}


void SetPWM(const long pwm_num, byte pwm_channel){
  if(pwm_channel==1){ // Right MOTOR
    analogWrite(pinMotorRight_forward, pwm_num);
    pwmRvalue = pwm_num;
  }
  else if(pwm_channel==2){ // Left MOTOR
    analogWrite(pinMotorLeft_forward, pwm_num);
    pwmLvalue = pwm_num;
  }
}

void go_Robot(){
  analogWrite(pinMotorRight_reverse, LOW);
  analogWrite(pinMotorRight_forward, pwmRvalue);
  analogWrite(pinMotorLeft_reverse, LOW);
  analogWrite(pinMotorLeft_forward, pwmLvalue);
  DPRINTLN("Start mowing");
  //mower_state = GO_FORWARD;
}

void stop_Robot(int delay_ms){
  analogWrite(pinMotorRight_reverse, LOW);
  analogWrite(pinMotorRight_forward, LOW);
  analogWrite(pinMotorLeft_reverse, LOW);
  analogWrite(pinMotorLeft_forward, LOW);
  delay(delay_ms);
}// void stopRobot(int delay_ms)

void poweroff(){
  stopMotors();
  bladesOFF();
  digitalWrite(pinLED_pause, HIGH);
  DPRINTLN("Power off");
  //mower_state = STOP;
}

//Return 1 or 0 from a random number.
int randomMove() {
  uint8_t rnd_number = random(1, 100);
  return rnd_number % 2;
}
