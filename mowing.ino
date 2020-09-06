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
        DPRINTLN("Move forward max Speed");
        _navState = CHECK_ALL;
      } break;
 
    case SPEED_DECREASE: {
        moveForward(minSpeed);
        DPRINTLN("Move forward low Speed");
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
        DPRINTLN("Move Left min Speed");
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
        DPRINTLN("Move right min Speed");
        if (isTimerPosition(MOVE_TO_NEW_POSITION)) {
          if (isObstacleRight == 1) _navState = RIGHT;
          else _navState = CHECK_ALL;
        }
      } break;
 
    case BACK: {
        moveBackward(minSpeed);
        DPRINTLN("Move backward min Speed");
        if (isTimerPosition(MOVE_TO_NEW_POSITION)) {
          if (randomMove() == 1)  _navState = LEFT; else  _navState = RIGHT;
        }
      } break;
  }
}


void cut_the_gras(){
  bladesON();
}
