void checkCurrent() {
  // Current Readings logic - used for checking if there is an
  // excessive current draw from one of the motors. If there is,
  // it's possible that the motors are drawing their stall current
  DPRINTLN("--------------------------------");

  LEFT_adcValue = analogRead(pinleftDriveCurrent);
  float LEFT_rawVoltage = (LEFT_adcValue / 1024.0) * 5000;    // Gets you mV
  float LEFT_Amps = ((LEFT_rawVoltage - offsetVoltage) / sensitivity);
  leftDriveCurrent = LEFT_Amps;
  DPRINT("LEFT Raw Sensor Value = " );
  DPRINTLN(LEFT_rawVoltage);
  DPRINT("LEFT Strom [mA]: ");
  DPRINTLN(LEFT_Amps, 3);

  RIGHT_adcValue = analogRead(pinrightDriveCurrent);
  float RIGHT_rawVoltage = (RIGHT_adcValue / 1024.0) * 5000;    // Gets you mV
  float RIGHT_Amps = ((RIGHT_rawVoltage - offsetVoltage) / sensitivity);
  rightDriveCurrent = RIGHT_Amps;
  DPRINT("RIGHT Raw Sensor Value = " );
  DPRINTLN(RIGHT_rawVoltage);
  DPRINT("RIGHT Strom [mA]: ");
  DPRINTLN(RIGHT_Amps, 3);

  BLADE_adcValue = analogRead(pinbladeCurrent);
  float BLADE_rawVoltage = (BLADE_adcValue / 1024.0) * 5000;    // Gets you mV
  float BLADE_Amps = ((BLADE_rawVoltage - offsetVoltage) / sensitivity);
  bladeCurrent = BLADE_Amps;
  DPRINT("BLADE Raw Sensor Value = " );
  DPRINTLN(BLADE_rawVoltage);
  DPRINT("BLADE Strom [mA]: ");
  DPRINTLN(BLADE_Amps, 3);
  DPRINTLN("--------------------------------");
}

void check_wheel_load () {
  if (leftDriveCurrent > Drivemaxleft || rightDriveCurrent > Drivemaxright) {  // High load, we are running in to something?
    // Add to load counter
    DriveCurrentcounter++;
    if (DriveCurrentcounter >= DriveCurrentcountermax) {
      startTimerPosition();
      DPRINTLN("High drive wheel load, turn around");
      DPRINTLN("MOTOR BACK");
      DriveCurrentcounter = 0;
      stop_Robot(200);
      mower_state = GO_BACKWARD;
    }
  }
}

void measureBattery() {
if (Battery_Monitor == 1 && _batt_timer == 10) {
  // read the value at analog input
  value_bat = analogRead(pinbatteryVoltage);
  vout = (value_bat * 4.9) / 1024.0;
  battv = vout / (R2 / (R1 + R2));
  DPRINT("BATTERIE Voltage = ");
  DPRINTLN(battv, 2);
      // Battery low?
      if (battv <= Battery_SwitchOffIfBelow) {
        Battery_Currentcounter++;
        if (Battery_Currentcounter >= Battery_Currentcountermax) {
          // Battery volt is to low!
          Serial.println("Battery low, stop!");
          LED_fullBat.off();
          LED_lowBat.on();
          LED_pause.on();
          stopMotors();
          bladesOFF();
          button_state = 0;
          mower_state = STOP;
          Battery_Currentcounter = 0;
          //set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Go to sleep to save power and stop execution
          //sleep_enable();
        }
      }
      _batt_timer = 0; // Reset counter
    }
    _batt_timer++;
}

    /*
      if (leftDriveCurrent > Drivemaxleft || rightDriveCurrent > Drivemaxright) {  // High load, we are running in to something?
      DriveCurrentcounter++;
      if (DriveCurrentcounter >= DriveCurrentcountermax) {
        if (leftDriveCurrent > Drivemaxleft) {          // Compare left motor current
          DPRINTLN("High LEFT wheel load, turn around");
          moveBackward(minSpeed);
          moveRight(minSpeed);
          DriveCurrentcounter = 0;
        }
        if (rightDriveCurrent > Drivemaxright) {       // Compare right motor current
          DPRINTLN("High RIGHT wheel load, turn around");
          moveBackward(minSpeed);
          moveLeft(minSpeed);
          DriveCurrentcounter = 0;
        }
      }
      }
    */
