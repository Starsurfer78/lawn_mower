/*
   Mowing Code based on https://www.intorobotics.com/obstacle-avoidance-robot/

   Ambrogio L50 Deluxe Lawn Mower Arduino Code
   Please make sure you have installed all the library files to the Arduino libraries folder
   All my work falls under the GNU Public License

*/

#include <NewPing.h>
#include <SimpleKalmanFilter.h>

//#define DEBUG  // Comment to disable debug serial output.
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
const int pinMotorLeft_forward = 6;    //(RPWM) to Arduino pin 6(PWM)
const int pinMotorLeft_reverse = 7;    //(LPWM) to Arduino pin 7(PWM)

long pwmLvalue = 255;   // Straight line speed Left Wheel (Looking from back of mower)
long pwmRvalue = 255;   // Straight line speed Right Wheel (Looking from back of mower)
byte pwmChannel;

uint8_t maximumSpeed = 255; //PWM value for maximum speed.
uint8_t minSpeed = 100; //PWM value for minimum speed.

// ------ Cutter Motor -------------------------------------
#define RPWM 8
#define L_EN 9
#define R_EN 10
int PWM_Blade_Speed            = 250;     // PWM signal for the cutter motor (speed of blade).
bool Cutting_Blades_Activate    = 1;      // Activates the cutting blades and disc in the code

// ------ battery -------------------------------------
#define batMonitor              0             // monitor battery and charge voltage?
#define batGoHomeIfBelow        23.7     // drive home voltage (Volt)
#define batSwitchOffIfBelow     21.7  // switch off battery if below voltage (Volt)
#define batSwitchOffIfIdle      1      // switch off battery if idle (minutes)
#define batFull                 29.4      // battery reference Voltage (fully charged) PLEASE ADJUST IF USING A DIFFERENT BATTERY VOLTAGE! FOR a 12V SYSTEM TO 14.4V
#define batChargingCurrentMax   1.6  // maximum current your charger can devliver
#define batFullCurrent          0.3      // current flowing when battery is fully charged
#define startChargingIfBelow    27.0 // start charging if battery Voltage is below
#define chargingTimeout         12600000 // safety timer for charging (ms) 12600000 = 3.5hrs

// ------ SONAR -------------------------------------
#define pinleft_Sensor_trigger   34
#define pinleft_Sensor_echo      35
#define pincenter_Sensor_trigger 36
#define pincenter_Sensor_echo    37
#define pinright_Sensor_trigger  38
#define pinright_Sensor_echo     39

#define SONAR_NUM 3          //The number of sensors.
#define MAX_DISTANCE 200     //Max distance to detect obstacles.
#define PING_INTERVAL 33     //Looping the pings after 33 microseconds.

uint8_t MIN_RANGE_OBSTACLE = 5;     //5 cm is the blind zone of the sensor.
uint8_t MAX_RANGE_OBSTACLE = 75;    //The maximum range to check if obstacle exists.

int LOOPING              = 10;    //Loop for every 10 milliseconds.
int DECREESE_SPEED_LOOP  = 400;   //Give some time to sensors for few more readings.
int MOVE_TO_NEW_POSITION = 500;   //Wait for the new position.

//
#define pinLED 13                  // LED
#define pinBuzzer 53               // Buzzer
#define pinButton 51               // digital ON/OFF button

// ------ Analog In pins -------------------------------------
#define pinbatteryVoltage = A0;   // battery voltage sensor
#define pinbladeCurrent = A1;
#define pinleftDriveCurrent = A2;
#define pinrightDriveCurrent = A3;

#define pinPerimeterRight A4    // perimeter NOT USED
#define pinPerimeterLeft A5     // perimeter NOT USED

// ------ Tilt Sensors -------------------------------------
#define pinTilt_Angle A8           // measures the angle of the mower
#define pinTilt_Orientation A9     // measures if the mower is upside down
/* SETTINGS END */

unsigned long _timerStart         = 0;
unsigned long _timerStartReady    = 0;
unsigned long _timerStartPosition = 0;

uint8_t oldSensorReading[3];    //Store last valid value of the sensors.

uint8_t leftSensor;             //Store the sensor's value.
uint8_t centerSensor;
uint8_t rightSensor;

bool isObstacleLeft;           //If obstacle detected or not.
bool isObstacleCenter;
bool isObstacleRight;

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

enum NavigationStates {
  CHECK_ALL,
  MAX_SPEED,
  SPEED_DECREASE,
  CHECK_OBSTACLE_POSITION,
  LEFT,
  CENTER,
  RIGHT,
  BACK
};
NavigationStates _navState = CHECK_ALL;

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

//Obstacle avoidance algorithm.
void obstacleAvoidance()
{
  switch (_navState) {
    case CHECK_ALL: { //if no obstacle, go forward at maximum speed
        if (isObstacleLeft == 0 && isObstacleCenter == 0 && isObstacleRight == 0) {
          _navState = MAX_SPEED;
        } else {
          startTimerReady();
          _navState = SPEED_DECREASE;
        }
      } break;

    case MAX_SPEED: {
        moveForward(maximumSpeed);
        _navState = CHECK_ALL;
      } break;

    case SPEED_DECREASE: {
        moveForward(minSpeed);
        //Wait for few more readings at low speed and then go to check the obstacle position
        if (isTimerReady(DECREESE_SPEED_LOOP)) _navState = CHECK_OBSTACLE_POSITION;
      } break;

    case CHECK_OBSTACLE_POSITION: {
        //If the path is free, go again to MAX_SPEED else check the obstacle position
        if (isObstacleLeft == 0 && isObstacleCenter == 0 && isObstacleRight == 0) {
          _navState = MAX_SPEED;
        }
        else if (isObstacleLeft == 1 && isObstacleCenter == 0 && isObstacleRight == 0) {
          startTimerPosition();
          _navState = LEFT;
        }
        else if (isObstacleLeft == 0 && isObstacleCenter == 1 && isObstacleRight == 0) {
          startTimerPosition();
          _navState = CENTER;
        }
        else if (isObstacleLeft == 0 && isObstacleCenter == 0 && isObstacleRight  == 1) {
          startTimerPosition();
          _navState = RIGHT;
        }
        else if (isObstacleLeft == 1 && isObstacleCenter == 1 && isObstacleRight == 1) {
          startTimerPosition();
          _navState = BACK;
        }
      } break;

    case LEFT: { //Move left and check obstacle. If obstacle exists, go again to left, else exit
        moveLeft(minSpeed);
        if (isTimerPosition(MOVE_TO_NEW_POSITION)) {
          if (isObstacleLeft == 1) _navState = LEFT;
          else _navState = CHECK_ALL;
        }
      } break;

    case CENTER: { //If obstacle exists, go left or right
        if (randomMove() == 1)  _navState = LEFT; else  _navState = RIGHT;
      } break;

    case RIGHT: {
        moveRight(minSpeed);
        if (isTimerPosition(MOVE_TO_NEW_POSITION)) {
          if (isObstacleRight == 1) _navState = RIGHT;
          else _navState = CHECK_ALL;
        }
      } break;

    case BACK: {
        moveBackward(minSpeed);
        if (isTimerPosition(MOVE_TO_NEW_POSITION)) {
          if (randomMove() == 1)  _navState = LEFT; else  _navState = RIGHT;
        }
      } break;
  }
}

void setup() {
  Serial.begin(115200);
  DPRINTLN("SETUP");
  // LED, buzzer, battery
  pinMode(pinLED, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  digitalWrite(pinBuzzer, 0);

  pingTimer[0] = millis() + 75;
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  setupMotors();
  stopMotors();
  setupCutter();
  stopCutter();
} // END SETUP

void loop() {
  if (isTimeForLoop(LOOPING)) {
    sensorCycle();
    applyKF();
    obstacleAvoidance();
    startTimer();
  }
} // END LOOP
