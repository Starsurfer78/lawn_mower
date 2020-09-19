NewPing sonar[SONAR_NUM] = {
  NewPing(pinleft_Sensor_trigger, pinleft_Sensor_echo, MAX_DISTANCE), // Trigger pin, echo pin, and max distance to ping.
  NewPing(pincenter_Sensor_trigger, pincenter_Sensor_echo, MAX_DISTANCE),
  NewPing(pinright_Sensor_trigger, pinright_Sensor_echo, MAX_DISTANCE)
};

/*
    stores a bool for each sensor (left, front, and right respectively
    true if nearest obstacle is within TURN_DIST
    true if not
*/
bool sensor[3] = {false, false, false};

// stores all possible sensor states
bool reactions[NUM_CASES][SONAR_NUM] = {
  {false, false, false}, // 0: no obstacles
  {false, true, false},  // 1: obstacle in front
  {false, true, true},   // 2: obstacles front and right
  {true, true, false},   // 3: obstacles front and left
  {true, true, true},    // 4: obstacles front, left, and right
  {true, false, true},   // 5: obstacles left and right
  {false, false, true},  // 6: obstacle to the right
  {true, false, false}
  }; // 7: obstacle to the left

void updateSensor() {
  for (int i = 0; i < SONAR_NUM; i++) {
    int dist = sonar[i].ping_cm();
    // if sonar returns 0 nearest obstacle is out of range
    if (dist == 0) sensor[i] = false;
    else sensor[i] = dist < TURN_DIST;
  }
}

int compareCases() {
  for (int i = 0; i < NUM_CASES; i++) {
    bool flag = true;
    for (int j = 0; j < SONAR_NUM; j++) {
      if (reactions[i][j] != sensor[j]) flag = false;
    }
    if (flag) return i;
  }
}

void checkSonar() {
  switch (compareCases()) {
    case 0: // no obstacles
      mower_state = GO_FORWARD;
      break;
    case 1: // obstacle in front
      startTimerPosition();
      mower_state = TURN_LEFT_90;
      break;
    case 2: // obstacles front and right
      startTimerPosition();
      mower_state = TURN_LEFT_90;
      break;
    case 3: // obstacles front and left
      startTimerPosition();
      mower_state = TURN_RIGHT_90;
      break;
    case 4: // obstacles front, left, and right      
      startTimerPosition();
      mower_state = TURN_LEFT_180;
      break;
    case 5: // obstacles left and right
      startTimerPosition();
      mower_state = TURN_LEFT_180;
      break;
    case 6: // obstacle to the right
      startTimerPosition();
      mower_state = TURN_LEFT_30;
      break;
    case 7: // obstacle to the left
      startTimerPosition();
      mower_state = TURN_RIGHT_30;
      break;
  }
}
