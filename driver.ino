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
}

void moveForward(uint8_t pwmValue) {
  analogWrite(pinMotorRight_reverse, LOW);
  analogWrite(pinMotorRight_forward, pwmValue);   //Motor 1
  analogWrite(pinMotorLeft_reverse, LOW);
  analogWrite(pinMotorLeft_forward, pwmValue);    //Motor 2
  DPRINTLN("Move Forward");
}

void moveBackward(uint8_t pwmValue) {
  analogWrite(pinMotorRight_forward, LOW);       //Motor 1
  analogWrite(pinMotorRight_reverse, pwmValue);
  analogWrite(pinMotorLeft_forward, LOW);        //Motor 2
  analogWrite(pinMotorLeft_reverse, pwmValue);
  DPRINTLN("Move Backward");
}

void moveLeft(uint8_t pwmValue) {
  analogWrite(pinMotorRight_reverse, LOW);
  analogWrite(pinMotorRight_forward, pwmValue);  //Right wheel forward.
  analogWrite(pinMotorLeft_forward, LOW);
  analogWrite(pinMotorLeft_reverse, pwmValue);   //Left wheel backward.
  DPRINTLN("Move LEFT");
}

void moveRight(uint8_t pwmValue) {
  analogWrite(pinMotorRight_forward, LOW);
  analogWrite(pinMotorRight_reverse, pwmValue);
  analogWrite(pinMotorLeft_reverse, LOW);
  analogWrite(pinMotorLeft_forward, pwmValue);
  DPRINTLN("Move RIGHT");
}

void leftOFF() {
  // Turns left motor OFF
  digitalWrite(pinMotorLeft_forward, LOW);
  digitalWrite(pinMotorLeft_reverse, LOW);
}

void rightOFF() {
  // Turns right motor OFF
  digitalWrite(pinMotorRight_forward, LOW);
  digitalWrite(pinMotorRight_reverse, LOW);
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
}

void poweroff(){
  stopMotors();
  bladesOFF();
  digitalWrite(pinLED_pause, HIGH);
  DPRINTLN("Power off");
}

//Return 1 or 0 from a random number.
int randomMove() {
  uint8_t rnd_number = random(1, 100);
  return rnd_number % 2;
}
