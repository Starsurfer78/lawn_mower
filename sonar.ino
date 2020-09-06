NewPing sonar[SONAR_NUM] = {
  NewPing(pinleft_Sensor_trigger, pinleft_Sensor_echo, MAX_DISTANCE), // Trigger pin, echo pin, and max distance to ping.
  NewPing(pincenter_Sensor_trigger, pincenter_Sensor_echo, MAX_DISTANCE),
  NewPing(pinright_Sensor_trigger, pinright_Sensor_echo, MAX_DISTANCE)
};

/*
  create Kalman filter objects for the sensors.
   SimpleKalmanFilter(e_mea, e_est, q);
   e_mea: Measurement Uncertainty
   e_est: Estimation Uncertainty
   q: Process Noise
*/
SimpleKalmanFilter KF_Left(2, 2, 0.01);
SimpleKalmanFilter KF_Center(2, 2, 0.01);
SimpleKalmanFilter KF_Right(2, 2, 0.01);

//looping the Sonar sensors
void sensorCycle() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle();
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
}

// If ping received, set the sensor distance to array.
void echoCheck() {
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

//Return the last valid value from the sensor.
void oneSensorCycle() {
  leftSensor   = returnLastValidRead(0, cm[0]);
  centerSensor = returnLastValidRead(1, cm[1]);
  rightSensor  = returnLastValidRead(2, cm[2]);
}

//If sensor value is 0, then return the last stored value different than 0.
int returnLastValidRead(uint8_t sensorArray, uint8_t cm) {
  if (cm != 0) {
    return oldSensorReading[sensorArray] = cm;
  } else {
    return oldSensorReading[sensorArray];
  }
}

//Apply Kalman Filter to sensor reading.
void applyKF() {
  isObstacleLeft = obstacleDetection(KF_Left.updateEstimate(leftSensor));
  isObstacleCenter = obstacleDetection(KF_Center.updateEstimate(centerSensor));
  isObstacleRight = obstacleDetection(KF_Right.updateEstimate(rightSensor));
}

//Define the minimum and maximum range of the sensors, and return true if an obstacle is in range.
bool obstacleDetection(int sensorRange) {
  if ((MIN_RANGE_OBSTACLE <= sensorRange) && (sensorRange <= MAX_RANGE_OBSTACLE)) return true; else return false;
}
