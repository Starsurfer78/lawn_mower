/*
   Mowing Code based on https://www.intorobotics.com/obstacle-avoidance-robot/

   Ambrogio L50 Deluxe Lawn Mower Arduino Code
   Please make sure you have installed all the library files to the Arduino libraries folder
   All my work falls under the GNU Public License

*/

#include <NewPing.h>              // https://github.com/microflo/NewPing/blob/master/NewPing.h
#include <SimpleKalmanFilter.h>   // https://github.com/denyssene/SimpleKalmanFilter
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
const int pinMotorLeft_forward = 6;    //(RPWM) to Arduino pin 6(PWM)
const int pinMotorLeft_reverse = 7;    //(LPWM) to Arduino pin 7(PWM)

long pwmLvalue = 255;   // Straight line speed Left Wheel (Looking from back of mower)
long pwmRvalue = 255;   // Straight line speed Right Wheel (Looking from back of mower)
byte pwmChannel;

uint8_t maximumSpeed = 255; //PWM value for maximum speed.
uint8_t minSpeed = 100;     //PWM value for minimum speed.

int Drivemaxleft = 250;     // Define variable for max motor current left and set default
int Drivemaxright = 250;    // Define variable for max motor current right and set default

// ------ Cutter Motor -------------------------------------
#define RPWM 8
#define L_EN 9
#define R_EN 10
int PWM_Blade_Speed = 250;    // PWM signal for the cutter motor (speed of blade).
int Blademax = 250;           // Define variable for max Blade current

bool Cutting_Blades_Activate    = 1;      // Activates the cutting blades and disc in the code

float leftDriveCurrent = 0.0;
float rightDriveCurrent = 0.0;
float bladeCurrent = 0.0;

// ------ SONAR -------------------------------------
#define pinleft_Sensor_trigger   34
#define pinleft_Sensor_echo      35
#define pincenter_Sensor_trigger 36
#define pincenter_Sensor_echo    37
#define pinright_Sensor_trigger  38
#define pinright_Sensor_echo     39

#define SONAR_NUM 3          //The number of Sonar sensors.
#define MAX_DISTANCE 200     //Max distance to detect obstacles.
#define PING_INTERVAL 33     //Looping the Sonar pings after 33 microseconds.

uint8_t MIN_RANGE_OBSTACLE = 5;     //5 cm is the blind zone of the sensor.
uint8_t MAX_RANGE_OBSTACLE = 75;    //The maximum range to check if obstacle exists.

// ------ Analog In pins -------------------------------------
#define pinbatteryVoltage A0   // battery voltage sensor
#define pinbladeCurrent A1
#define pinleftDriveCurrent A2
#define pinrightDriveCurrent A3

// ------ Perimeter Sensor -------------------------------------
#define pinPerimeterRight A4    // perimeter NOT USED
#define pinPerimeterLeft A5     // perimeter NOT USED

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

// ------ battery -------------------------------------
bool Battery_Monitor                = 0;            // monitor battery and charge voltage?
float Battery_Max                   = 29.4;         // battery reference Voltage (fully charged) PLEASE ADJUST IF USING A DIFFERENT BATTERY VOLTAGE! FOR a 12V SYSTEM TO 14.4V
float Battery_GoHomeIfBelow         = 23.7;         // drive home voltage (Volt)
float Battery_SwitchOffIfBelow      = 21.7;         // switch off battery if below voltage (Volt)
float Battery_SwitchOffIfIdle       = 1;            // switch off battery if idle (minutes)
float Battery_ChargingCurrentMax    = 1.6;          // maximum current your charger can devliver
float Battery_ChargingFullCurrent   = 0.3;          // current flowing when battery is fully charged
float Battery_startChargingIfBelow  = 27.0;         // start charging if battery Voltage is below
float Battery_chargingTimeout       = 12600000;     // safety timer for charging (ms) 12600000 = 3.5hrs

// ------ ASC712 Current Sensor (30A) -------------------------------------
int sensitivity = 66;
int adcValue = 0;
int offsetVoltage = 2500;
double adcVoltage = 0;
double currentValue = 0;

int LOOPING              = 10;    //Loop for every 10 milliseconds.
int DECREESE_SPEED_LOOP  = 400;   //Give some time to sensors for few more readings.
int MOVE_TO_NEW_POSITION = 500;   //Wait for the new position.

int Mower_Turn_Delay_Min  = 1000;   // Min Max Turn time of the Mower after it reverses at the wire.
int Mower_Turn_Delay_Max  = 2500;   // A random turn time between these numbers is selected by the software
int Mower_Reverse_Delay   = 1800;   // Time the mower revreses at the wire

int DriveCurrentcounter = 0;      // Counter that gets added while high load. Used for not giving up directly on high load
int DriveCurrentcountermax = 10;  //Used for not giving up directly on high load

/* SETTINGS END */

unsigned long _timerStart         = 0;
unsigned long _timerStartReady    = 0;
unsigned long _timerStartPosition = 0;

// Interval for checking Motors Currents and Battery Voltage
int _batt_timer = 0;

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
  BACK,
  START,
  STOP
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

int state = 0;

void setup() {
  Serial.begin(115200);
  DPRINTLN("SETUP");
  // LED, buzzer, battery
  LED_on.on();
  //pinMode(pinLED_on, OUTPUT);
  //digitalWrite(pinLED_on, HIGH);
  LED_pause.off();
  //pinMode(pinLED_pause, OUTPUT);
  //digitalWrite(pinLED_pause, 0);
  LED_lowBat.off();
  //pinMode(pinLED_lowBat, OUTPUT);
  //digitalWrite(pinLED_lowBat, 0);
  LED_fullBat.off();
  //pinMode(pinLED_fullBat, OUTPUT);
  //digitalWrite(pinLED_fullBat, 0);

  pinMode(pinBuzzer, OUTPUT);
  digitalWrite(pinBuzzer, 0);

  pinMode(pinleftDriveCurrent, INPUT);
  pinMode(pinrightDriveCurrent, INPUT);
  pinMode(pinbladeCurrent, INPUT);

  pingTimer[0] = millis() + 75;
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  setupMotors();
  stopMotors();
  setupBlades();
  bladesOFF();
  // set initial state
  _navState = START;
} // END SETUP

void loop() {

  if (Button_startstop.onPressed()) {
    state++;
    if (state == 1) {
      LED_pause.off();

      if (isTimeForLoop(LOOPING)) {
        // Check battery voltage
        if (_batt_timer == 10) { // Dont check battery every time
          // Measure Current
          leftDriveCurrent = measureCurrent(pinleftDriveCurrent);
          rightDriveCurrent = measureCurrent(pinrightDriveCurrent);
          bladeCurrent = measureCurrent(pinbladeCurrent);
          /*
          DPRINT("leftDriveCurrent: ");
          DPRINT(leftDriveCurrent);
          DPRINTLN(" Amp");

          DPRINT("rightDriveCurrent: ");
          DPRINT(rightDriveCurrent);
          DPRINTLN(" Amp");

          DPRINT("bladeCurrent: ");
          DPRINT(bladeCurrent);
          DPRINTLN("Amp");
          */
          
          /*
                if (leftDriveCurrent > Drivemaxleft || rightDriveCurrent > Drivemaxright || bladeCurrent > Blademax) {
                  //stopMotors();
                  //bladesOFF();
                  _navState = STOP;
                  DPRINT("ERROR: Overload");
                }
          */
          if (leftDriveCurrent > Drivemaxleft || rightDriveCurrent > Drivemaxright) {  // High load, we are running in to something?
            // Add to load counter
            DriveCurrentcounter++;
            if (DriveCurrentcounter >= DriveCurrentcountermax) {
              // Turn around
              DPRINTLN("High drive wheel load, turn around");
              _navState = BACK;
              DriveCurrentcounter = 0;
            }
          }
          /*
            // Battery low?
            if (battv <= 215) {
            // Battery volt is to low!
            Serial.println("Battery low, stop!");
            LED_fullBat.off();
            pinLED_lowBat.on();
            state = 0;
            set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Go to sleep to save power and stop execution
            sleep_enable();
            }
          */
          _batt_timer = 0; // Reset counter
        }// Check battery voltage END

        _batt_timer = _batt_timer + 1;
        // Debug
        DPRINT("batt_timer: ");
        DPRINTLN(_batt_timer);

        sensorCycle();
        applyKF();
        obstacleAvoidance();
        startTimer();
      }
      else {
        state = 0;
        stopMotors();
        bladesOFF();
        LED_pause.on();
      }
    }
  }//Button Start/Stop
} // END LOOP
