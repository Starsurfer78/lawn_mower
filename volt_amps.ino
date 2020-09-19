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

void measureBattery() {
  // read the value at analog input
  value_bat = analogRead(pinbatteryVoltage);
  vout = (value_bat * 4.9) / 1024.0;
  battv = vout / (R2 / (R1 + R2));

  DPRINT("BATTERIE Voltage = ");
  DPRINTLN(battv, 2);
  _batt_timer = 0; // Reset counter
}
