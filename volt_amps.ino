void checkCurrent() {
  // Current Readings logic - used for checking if there is an
  // excessive current draw from one of the motors. If there is,
  // it's possible that the motors are drawing their stall current
  leftDriveCurrent = measureCurrent(pinleftDriveCurrent);
  rightDriveCurrent = measureCurrent(pinrightDriveCurrent);
  bladeCurrent = measureCurrent(pinbladeCurrent);
  if (pinleftDriveCurrent > Drivemaxleft || pinrightDriveCurrent > Drivemaxright || pinbladeCurrent > Blademax) {
    //stopMotors();
    //bladesOFF();
  }
}

int measureCurrent(int currentSenseAnalogPin) {
  //  return analogInputToVolts(analogRead(currentSenseAnalogPin)) / 0.13;
  adcValue = analogRead(currentSenseAnalogPin);
  adcVoltage = (adcValue / 1024.0) * 5000;
  currentValue = ((adcVoltage - offsetVoltage) / sensitivity);
  DPRINT("Raw Sensor Value = " );
  DPRINTLN(adcValue);
  DPRINT("Voltage(mV) = ");
  DPRINTLN(adcVoltage, 3);
  DPRINT("Current = ");
  DPRINTLN(currentValue, 3);
  return currentValue;
}
