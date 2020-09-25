/*
  Obstacle avoidance example
  by Andrew Kramer
  8/3/2015

  Uses 3 ultrasonic sensors to determine
  if nearest obstacle is too close.
  Outlines 8 possible cases for
  positions of obstacles.
*/
/*
   Mowing Code based on https://www.intorobotics.com/obstacle-avoidance-robot/

   Ambrogio L50 Deluxe Lawn Mower Arduino Code
   Please make sure you have installed all the library files to the Arduino libraries folder
   All my work falls under the GNU Public License


  Der L50 kennt folgende Spannungszustände:
  Ladespannung liegt an den Ladekontakten an = HIGH BATTERY (grün) blinkt -> Ladung
  Spannung > 24,9 Volt = HIGH BATTERY (grün) leuchtet -> Normalbetrieb
  Spannung >22,5V  und <=24,9V = LOW BATTERY blinkt -> Robi bleibt stehen
  Spannung <22,5V = LOW BATTERY (brennt) -> Robi schaltet sich ab!

  Ambrogio L50 Tasterbelegung (d.h. die Belegung am Original L50-Board).
  Taster also an Pin3 anklemmen,
  Masse an Pin4.
  (eigene LEDs ggf. an Pin1 bzw. Pin2
  die LEDs am Arduino nur mit Vorwiderstand z.B. 220 Ohm benutzen).
  https://forum.ardumower.de/data/media/kunena/attachments/905/h4511ad4.png/

*/

#include <NewPing.h>              // https://github.com/microflo/NewPing/blob/master/NewPing.h
#include <RBD_Timer.h>            // https://github.com/alextaujenis/RBD_Timer
#include <RBD_Button.h>           // https://github.com/alextaujenis/RBD_Button
#include <RBD_Light.h>            // https://github.com/alextaujenis/RBD_Light

#define DEBUG  // Comment to disable debug serial output.
#ifdef DEBUG
#define DPRINT(...)    Serial.print(__VA_ARGS__)
#define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#else
#define DPRINT(...)
#define DPRINTLN(...)
#endif

// ------ SETTINGS CHANGE IF YOU NEED -------------------------------------
/*BTS7960 Motor Driver Carrier*/
//const int MotorRight_R_EN = 8;    (VCC) to Arduino 5V pin
//const int MotorRight_L_EN = 9;    (VCC) to Arduino 5V pin
const int pinMotorRight_forward = 2;   //(RPWM) to Arduino pin 2(PWM)
const int pinMotorRight_reverse = 3;   //(LPWM) to Arduino pin 3(PWM)

//const int MotorLeft_R_EN = 10;     (VCC) to Arduino 5V pin
//const int MotorLeft_L_EN = 11;     (VCC) to Arduino 5V pin
const int pinMotorLeft_forward = 4;    //(RPWM) to Arduino pin 6(PWM)
const int pinMotorLeft_reverse = 5;    //(LPWM) to Arduino pin 7(PWM)

long pwmLvalue = 250;   // Straight line speed Left Wheel (Looking from back of mower)
long pwmRvalue = 250;   // Straight line speed Right Wheel (Looking from back of mower)
byte pwmChannel;

uint8_t maximumSpeed = 250; //PWM value for maximum speed.
uint8_t minSpeed = 140;     //PWM value for minimum speed.

// ------ Cutter Motor -------------------------------------
#define RPWM 7
#define L_EN 11    //(VCC) to Arduino 5V pin
#define R_EN 10   //(VCC) to Arduino 5V pin
int PWM_Blade_Speed = 250;    // PWM signal for the cutter motor (speed of blade).
int Blademax = 250;           // Define variable for max Blade current

bool Cutting_Blades_Activate    = 1;      // Activates the cutting blades and disc in the code

// ------ SONAR -------------------------------------
#define pinleft_Sensor_trigger   34
#define pinleft_Sensor_echo      40
#define pincenter_Sensor_trigger 36
#define pincenter_Sensor_echo    42
#define pinright_Sensor_trigger  38
#define pinright_Sensor_echo     44

#define TURN_DIST 35 // distance at which the bot will turn
#define MAX_DISTANCE 200 // max range of sonar sensors
#define SONAR_NUM 3 // number of sonar sensors
#define NUM_CASES 8 // number of reaction cases

// ------ Analog In pins -------------------------------------
#define pinbatteryVoltage A4   // battery voltage sensor
#define pinbladeCurrent A1
#define pinleftDriveCurrent A2
#define pinrightDriveCurrent A3

// ------ Perimeter Sensor -------------------------------------
//#define pinPerimeterRight A4    // perimeter NOT USED
//#define pinPerimeterLeft A5     // perimeter NOT USED

// ------ Tilt Sensors -------------------------------------
#define pinTilt_Angle A8           // measures the angle of the mower
#define pinTilt_Orientation A9     // measures if the mower is upside down

// ------ LEDs -------------------------------------
#define pinLED_on 13                  // LED on
#define pinLED_pause 14                  // LED
#define pinLED_lowBat 15                  // LED
#define pinLED_fullBat 16                  // LED
RBD::Light  LED_on(pinLED_on);
RBD::Light  LED_pause(pinLED_pause);
RBD::Light  LED_lowBat(pinLED_lowBat);
RBD::Light  LED_fullBat(pinLED_fullBat);

// ------ Buzzer -------------------------------------
#define pinBuzzer 53               // Buzzer

// ------ Buttons -------------------------------------
#define pinButton_onoff 50               // digital ON/OFF button
#define pinButton_startstop 51               // digital ON/OFF button
RBD::Button Button_onoff(pinButton_onoff); // input_pullup on digital pin
RBD::Button Button_startstop(pinButton_startstop); // input_pullup on digital pin

int button_state = 0;

// ------ battery -------------------------------------
bool Battery_Monitor                = 1;            // monitor battery and charge voltage?
float Battery_Max                   = 29.4;         // battery reference Voltage (fully charged) PLEASE ADJUST IF USING A DIFFERENT BATTERY VOLTAGE! FOR a 12V SYSTEM TO 14.4V
float Battery_GoHomeIfBelow         = 24.5;         // drive home voltage (Volt)
float Battery_SwitchOffIfBelow      = 24.4;         // switch off battery if below voltage (Volt)
float Battery_SwitchOffIfIdle       = 1;            // switch off battery if idle (minutes)
float Battery_ChargingCurrentMax    = 1.6;          // maximum current your charger can devliver
float Battery_ChargingFullCurrent   = 0.3;          // current flowing when battery is fully charged
float Battery_startChargingIfBelow  = 27.0;         // start charging if battery Voltage is below
float Battery_chargingTimeout       = 12600000;     // safety timer for charging (ms) 12600000 = 3.5hrs

int _batt_timer = 0;                 // Interval for checking Motors Currents and Battery Voltage
int Battery_Currentcounter = 0;      // Counter that gets added while high load. Used for not giving up directly on high load
int Battery_Currentcountermax = 10;  //Used for not giving up directly on high load

// ------ voltage divider Battery -------------------------------------
float vout = 0.0;
float battv = 0.0;
float R1 = 100000.0; //100k
float R2 = 10000.0; //10k
int value_bat = 0;

// ------ ASC712 Current Sensor (30A) -------------------------------------
int sensitivity = 66;   // use 100 for 20A Module and 66 for 30A Module
const int zeroCurrentValue = 510.8;
int LEFT_adcValue = 0;
int RIGHT_adcValue = 0;
int BLADE_adcValue = 0;
int offsetVoltage = 2500;
double rawVoltage = 0;    //voltage measuring
double LEFT_Amps = 0;      // Current measuring
double RIGHT_Amps = 0;      // Current measuring
double BLADE_Amps = 0;      // Current measuring

float leftDriveCurrent = 0.0;
float rightDriveCurrent = 0.0;
float bladeCurrent = 0.0;

int Drivemaxleft = 3.5;           // Define variable for max motor current left and set default
int Drivemaxright = 3.5;          // Define variable for max motor current right and set default
int DriveCurrentcounter = 0;      // Counter that gets added while high load. Used for not giving up directly on high load
int DriveCurrentcountermax = 8;  //Used for not giving up directly on high load
/* SETTINGS END */


// Movement Stats
typedef enum {
  GO_FORWARD         = 0,
  GO_BACKWARD        = 1,
  TURN_LEFT_30          = 2,
  TURN_LEFT_90          = 3,
  TURN_LEFT_180          = 4,
  TURN_RIGHT_30         = 5,
  TURN_RIGHT_90         = 6,
  TURN_RIGHT_180         = 7,
  STOP               = 8
} state;

// Inital State
state mower_state = STOP;

int LOOPING              = 50;      // Loop for every 10 milliseconds.
int MOVE_TURN_DELAY_MIN  = 1000;    // Min Max Turn time of the Mower after it reverses at the wire.
int MOVE_TURN_DELAY_MAX  = 2500;    // A random turn time between these numbers is selected by the software
int MOVE_TO_NEW_POSITION = 5000;    // Wait for the new position.
int TIME_GO_BACKWARD = 1000;        // Time the mower revreses.
int TIME_TURN_30 = 300;             // Wait to turn the mower 30 Degree.
int TIME_TURN_90 = 900;             // Wait to turn the mower 90 Degree.
int TIME_TURN_180 = 1400;           // Wait to turn the mower 180 Degree.

unsigned long _timerStart         = 0;
unsigned long _timerStartReady    = 0;
unsigned long _timerStartPosition = 0;

// Time Helpers
void startTimer() {
  _timerStart = millis();
}

void startTimerReady() {
  _timerStartReady = millis();
}

void startTimerPosition() {
  _timerStartPosition = millis();
}

bool isTimeForLoop(int _mSec) {
  return (millis() - _timerStart) > _mSec;
}

bool isTimerReady(int _mSec) {
  return (millis() - _timerStartReady) > _mSec;
}

bool isTimerPosition(int _mSec) {
  return (millis() - _timerStartPosition) > _mSec;
}

// SETUP
void setup() {
  Serial.begin(115200);
  DPRINTLN("SETUP");
  // LED, buzzer, battery
  LED_on.on();
  LED_pause.on();
  LED_lowBat.off();
  LED_fullBat.on();

  pinMode(pinBuzzer, OUTPUT);
  digitalWrite(pinBuzzer, 0);

  pinMode(pinleftDriveCurrent, INPUT);
  pinMode(pinrightDriveCurrent, INPUT);
  pinMode(pinbladeCurrent, INPUT);

  pinMode(pinbatteryVoltage, INPUT);

  setupMotors();
  stopMotors();
  setupBlades();
  bladesOFF();
  button_state = 0;
  DPRINT("Button STATE: ");
  DPRINTLN(button_state);
  DPRINTLN("Setup ENDE");
  DPRINTLN("----------------------------");
} // SETUP END

// LOOP START
void loop() {

  if (button_state == 0) {
    button_state = 0;
    stopMotors();
    bladesOFF();
    measureBattery();
    LED_pause.on();
    mower_state = STOP;
  }

  //DPRINTLN("-----------------------------");
  //DPRINT("mower_state:");
  //DPRINTLN(mower_state);
  //DPRINTLN("-----------------------------");

  if (Button_startstop.onPressed()) {
    button_state++;
    DPRINT("Button STATE: ");
    DPRINTLN(button_state);
    mower_state = GO_FORWARD;
    DPRINT("mower_state:");
    DPRINTLN(mower_state);
    delay(200);
  }

  if (button_state == 1) {
    LED_pause.off();
    bladesON();
    // Check battery voltage
    measureBattery();
    checkCurrent();
    check_wheel_load();
    updateSensor();
    if (mower_state != 8 && mower_state == 0) {
      checkSonar();
    }

    //Movements
    //0:  GO_FORWARD
    if (mower_state != 8 && mower_state == 0) {
      go_Robot();
    }
    // 1:  GO_BACKWARD
    else if (mower_state == 1) {
      moveBackward (minSpeed, minSpeed);
      DPRINTLN("GO_BACKWARD");
      if (isTimerPosition(TIME_GO_BACKWARD)) {
        //mower_state = GO_FORWARD;
        //if (randomMove() == 1)  mower_state = TURN_RIGHT_90; else  mower_state = TURN_LEFT_90;
        mower_state = TURN_LEFT_90;
      }
    }
    // 2: TURN_LEFT_30
    else if (mower_state == 2) {
      moveLeft(maximumSpeed);
      DPRINTLN("TURN_LEFT_30");
      if (isTimerPosition(TIME_TURN_30)) {
        mower_state = GO_FORWARD;
      }
    }
    // 3: TURN_LEFT_90
    else if (mower_state == 3) {
      moveLeft(maximumSpeed);
      DPRINTLN("TURN_LEFT_90");
      if (isTimerPosition(TIME_TURN_90)) {
        mower_state = GO_FORWARD;
      }
    }
    // 4:  TURN_LEFT_180
    else if (mower_state == 4) {
      moveLeft(maximumSpeed);
      DPRINTLN("TURN_LEFT_180");
      if (isTimerPosition(TIME_TURN_180)) {
        mower_state = GO_FORWARD;
      }
    }
    // 5: TURN_RIGHT_30
    else if (mower_state == 5) {
      moveRight(maximumSpeed);
      DPRINTLN("TURN_RIGHT_30");
      if (isTimerPosition(TIME_TURN_30)) {
        mower_state = GO_FORWARD;
      }
    }
    // 6: TURN_RIGHT_90
    else if (mower_state == 6) {
      moveRight(maximumSpeed);
      DPRINTLN("TURN_RIGHT_90");
      if (isTimerPosition(TIME_TURN_90)) {
        mower_state = GO_FORWARD;
      }
    }
    // 7: TURN_RIGHT_180
    else if (mower_state == 7) {
      moveRight(maximumSpeed);
      DPRINTLN("TURN_RIGHT_180");
      if (isTimerPosition(TIME_TURN_180)) {
        mower_state = GO_FORWARD;
      }
    }
    // 8: STOP
    else if (mower_state == 8) {
      stopMotors();
      DPRINTLN("STOP Mower");
    }
    startTimer();
  }// if state=1

  if (button_state == 2) {
    button_state = 0;
    stopMotors();
    bladesOFF();
    LED_pause.on();
    mower_state = STOP;

  }
} // END LOOP
