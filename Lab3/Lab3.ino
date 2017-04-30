#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <Servo.h>
#include <QueueArray.h>

#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define RED 0x1
#define GREEN 0x2
#define YELLOW 0x3
#define BLUE 0x4
#define WHITE 0x7
#define SENSOR_OUT 8
#define FRONT_SENSOR A0
#define LEFT_SENSOR A1
#define RIGHT_SENSOR A2
#define LEFT_ENCODER_PIN 10
#define RIGHT_ENCODER_PIN 11
#define SENSOR_SENSITIVITY 11

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
Servo LServo, RServo;

enum DIRECTIONS { NORTH, EAST, SOUTH, WEST, NONE };
enum FLOW_STATE { SETUP_ONE, PATH_PLANNING, GET_TO_GOAL, SETUP_TWO, SHORTEST_PATH, FINISH_STATE, ERROR_STATE };
enum CELL_STATE { U, V };
enum WALL_STATE { N, W };

int setup_option = 0,
    setup_direction = NORTH,
    setup_start_location = 0,
    setup_end_location = 0,
    robot_state = SETUP_ONE,
    current_row = 0,
    current_col = 0,
    current_direction = EAST,
    target_row = 0,
    target_col = 0,
    left_encoder_last = 0,
    right_encoder_last = 0,
    target_encoder_count,
    left_encoder_count,
    right_encoder_count,
    front_distance,
    left_distance,
    right_distance;

bool looking_for_color = false;

struct cell {
  bool state, north, south, west, east;
};

cell maze[4][4] = {
  {
    { U, W, N, W, N },
    { U, W, N, N, N },
    { U, W, N, N, N },
    { U, W, N, N, W }
  }, {
    { U, N, N, W, N },
    { U, N, N, N, N },
    { U, N, N, N, N },
    { U, N, N, N, W }
  }, {
    { U, N, N, W, N },
    { U, N, N, N, N },
    { U, N, N, N, N },
    { U, N, N, N, W }
  }, {
    { U, N, W, W, N },
    { U, N, W, N, N },
    { U, N, W, N, N },
    { U, N, W, N, W }
  }
};

int brushfire_maze[4][4] = { 0 };
QueueArray<int> queue;

void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SENSOR_OUT, INPUT);
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  LServo.attach(2);
  RServo.attach(3);
  LServo.writeMicroseconds(1500);
  RServo.writeMicroseconds(1500);

  Serial.begin(9600);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setBacklight(WHITE);
}

void loop() {
  switch(robot_state) {
    case SETUP_ONE:
      runInitialSetup();
      break;
    case PATH_PLANNING:
      runPathPlanning();
      break;
    case GET_TO_GOAL:
      runShortestPath();
      break;
    case SETUP_TWO:
      runSecondSetup();
      break;
    case SHORTEST_PATH:
      runShortestPath();
      break;
    case FINISH_STATE:
      printHelper("ALL DONE", "WEW");
      delay(10000);
      break;
    case ERROR_STATE:
    default:
      printErrorMessage();
  }
}

void printMaze() {
  lcd.clear();
  lcd.setCursor(0, 0);
  for (unsigned char i = 0; i < 4; i++) {
    for (unsigned char j = 0; j < 4; j++) {
      if (maze[i][j].state) {
        lcd.print("X");
      } else {
        lcd.print("O");
      }
    }
  }
  lcd.setCursor(0, 1);
  lcd.print("G");
  unsigned char gridPos = (current_row * 4) + current_col;
  lcd.print(gridPos);
  lcd.print(" ");
  bool frontWall = checkFrontWall();
  bool leftWall = checkLeftWall();
  bool rightWall = checkRightWall();
  switch(current_direction) {
    case NORTH: {
      lcd.print("W");
      if (leftWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("N");
      if (frontWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("E");
      if (rightWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("SU");
      break;
    }
    case EAST: {
      lcd.print("WU ");
      lcd.print("N");
      if (leftWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("E");
      if (frontWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("S");
      if (rightWall) lcd.print("X");
      else lcd.print("O");
      break;
    }
    case SOUTH: {
      lcd.print("W");
      if (rightWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("NU ");
      lcd.print("E");
      if (leftWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("S");
      if (frontWall) lcd.print("X");
      else lcd.print("O");
      break;
    }
    case WEST: {
      lcd.print("W");
      if (frontWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("N");
      if (rightWall) lcd.print("X ");
      else lcd.print("O ");
      lcd.print("EU ");
      lcd.print("S");
      if (leftWall) lcd.print("X ");
      else lcd.print("O ");
      break;
    }
    default: {
      lcd.print("\\/(o.o)\\/");
    }
  }
}

void printHelper(String msg, int info) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(msg);
  lcd.setCursor(0, 1);
  lcd.print(info);
}

void printHelper(String msg, String info) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(msg);
  lcd.setCursor(0, 1);
  lcd.print(info);
}

void printErrorMessage() {
  printHelper("ERROR_STATE", "p(u.u)o");
  delay(5000);
}

int updateValue(int value, int range, int change) {
  return (value + range + change) % range;
}

String getDirection(int direction_to_check) {
  switch(direction_to_check) {
    case NORTH: return "NORTH";
    case EAST: return "EAST";
    case WEST: return "WEST";
    case SOUTH: return "SOUTH";
    default: return "NONE";
  }
}

void buttonsSetup(int &value, int range) {
  uint8_t buttons;
  while(!(buttons = lcd.readButtons())) {}
  if (buttons) {
    if (buttons & BUTTON_UP) {
      value = updateValue(value, range, 1);
    }
    if (buttons & BUTTON_DOWN) {
      value = updateValue(value, range, -1);
    }
    if (buttons & BUTTON_LEFT) {
      setup_option = updateValue(setup_option, 3, -1);
    }
    if (buttons & BUTTON_RIGHT) {
      setup_option = updateValue(setup_option, 3, 1);
    }
    if (buttons & BUTTON_SELECT) {
      setup_option = 3;
    }
  }
}

void setupStartLocation() {
  printHelper("Start Location?", setup_start_location);
  buttonsSetup(setup_start_location, 16);
}

void setupEndLocation() {
  printHelper("End Location?", setup_end_location);
  buttonsSetup(setup_end_location, 16);
}

void setupStartDirection() {
  printHelper("Start Direction?", getDirection(setup_direction));
  buttonsSetup(setup_direction, 4);
}

void finishInitialSetup() {
  current_direction = setup_direction;
  current_row = setup_start_location / 4;
  current_col = setup_start_location % 4;
  target_row = setup_end_location / 4;
  target_col = setup_end_location % 4;
  robot_state = PATH_PLANNING;
  maze[current_row][current_col].state = V;
}

void runInitialSetup() {
  switch(setup_option) {
    case 0:
      setupStartLocation();
      break;
    case 1:
      setupEndLocation();
      break;
    case 2:
      setupStartDirection();
      break;
    case 3:
      finishInitialSetup();
      break;
    default:
      robot_state = ERROR_STATE;
  }
  delay(200);
}

void runSecondSetup() {
  printHelper("GO TO START", "Hit Select");
  uint8_t buttons;
  while(!(buttons = lcd.readButtons())) {}
  if (buttons && BUTTON_SELECT) {
    brushfire();
    current_direction = setup_direction;
    current_row = setup_start_location / 4;
    current_col = setup_start_location % 4;
    for (unsigned char i = 0; i < 4; i++) {
      for (unsigned char j = 0; j < 4; j++) {
        maze[i][j].state = U;
      }
    }
    robot_state = SHORTEST_PATH;
  }
}

void readSensorValues() {
  float front = analogRead(FRONT_SENSOR);
  float right = analogRead(RIGHT_SENSOR);
  float left = analogRead(LEFT_SENSOR);
  front_distance = 500*pow(front, -0.85);
  right_distance = 500*pow(right, -0.85);
  left_distance = 500*pow(left, -0.85);
}

void readAvgSensorValues(int d = 100) {
  float front[15], right[15], left[15];
  for(int x = 0; x < 15; x++) {
    front[x] = analogRead(FRONT_SENSOR);
    right[x] = analogRead(RIGHT_SENSOR);
    left[x] = analogRead(LEFT_SENSOR);
    delay(d);
  }
  float tmp;
  for(unsigned char i = 0; i < 15; i++) {
    for(unsigned char j = 0; j < 14; j++) {
      if(front[j] < front[j+1]) {
        tmp = front[j];
        front[j] = front[j+1];
        front[j+1] = tmp;
      }
      if(left[j] < left[j+1]) {
        tmp       = left[j];
        left[j]    = left[j+1];
        left[j+1]  = tmp;
      }
      if(right[j] < right[j+1]) {
        tmp       = right[j];
        right[j]    = right[j+1];
        right[j+1]  = tmp;
      }
    }
  }
  int mdn_front = front[7], mdn_left = left[7], mdn_right = right[7];
  front_distance = 500*pow(mdn_front, -0.85);
  right_distance = 500*pow(mdn_right, -0.85);
  left_distance = 500*pow(mdn_left, -0.85);
}

void updateLcdColor() {
  switch(current_direction) {
    case NORTH:
      lcd.setBacklight(BLUE);
      break;
    case EAST:
      lcd.setBacklight(RED);
      break;
    case SOUTH:
      lcd.setBacklight(YELLOW);
      break;
    case WEST:
      lcd.setBacklight(GREEN);
      break;
  }
}

void readColors() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);

  int fRed = pulseIn(SENSOR_OUT, LOW);

  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);

  int fGreen = pulseIn(SENSOR_OUT, LOW);

  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);

  int fBlue = pulseIn(SENSOR_OUT, LOW);

  if (looking_for_color && (fRed < 240 || fBlue < 240)) {
    updateLcdColor();
    looking_for_color = false;
  } else if (!looking_for_color && !(fRed < 240 || fBlue < 240)) {
    lcd.setBacklight(WHITE);
  }
}

void markBoard() {
  if (current_direction == NORTH) current_row -= 1;
  if (current_direction == EAST) current_col += 1;
  if (current_direction == SOUTH) current_row += 1;
  if (current_direction == WEST) current_col -= 1;
  maze[current_row][current_col].state = V;
}

void getVisitedCount() {
  int count = 0;
  for (unsigned char i = 0; i < 4; i++) {
    for (unsigned char j = 0; j < 4; j++) {
      if (maze[i][j].state) count += 1;
    }
  }
  if (count == 16) {
    readAvgSensorValues();
    checkFrontWall();
    checkLeftWall();
    checkRightWall();
    printMaze();
    brushfire();
    robot_state = GET_TO_GOAL;
  }
}

bool checkWall(int direction_to_check, int distance_to_check) {
  bool isWall = distance_to_check < SENSOR_SENSITIVITY && distance_to_check > 0;
  switch(direction_to_check) {
    case NORTH:
      if (current_row == 0) return true;
      if (isWall && !maze[current_row][current_col].north) {
        maze[current_row][current_col].north = W;
        maze[current_row - 1][current_col].south = W;
      }
      break;
    case EAST:
      if (current_col == 3) return true;
      if (isWall && !maze[current_row][current_col].east) {
        maze[current_row][current_col].east = W;
        maze[current_row][current_col + 1].west = W;
      }
      break;
    case SOUTH:
      if (current_row == 3) return true;
      if (isWall && !maze[current_row][current_col].south) {
        maze[current_row][current_col].south = W;
        maze[current_row + 1][current_col].north = W;
      }
      break;
    case WEST:
      if (current_col == 0) return true;
      if (isWall && !maze[current_row][current_col].west) {
        maze[current_row][current_col].west = W;
        maze[current_row][current_col - 1].east = W;
      }
      break;
  }
  return isWall;
}

bool checkFrontWall() {
  return checkWall(current_direction, front_distance);
}

bool checkLeftWall() {
  return checkWall(updateValue(current_direction, 4, -1), left_distance);
}

bool checkRightWall() {
  return checkWall(updateValue(current_direction, 4, 1), right_distance);
}

bool checkVisited(int direction_to_check) {
  switch(direction_to_check) {
    case NORTH:
      return current_row == 0 || maze[current_row - 1][current_col].state;
    case EAST:
      return current_col == 3 || maze[current_row][current_col + 1].state;
    case SOUTH:
      return current_row == 3 || maze[current_row + 1][current_col].state;
    case WEST:
      return current_col == 0 || maze[current_row][current_col - 1].state;
    default:
      return true;
  }
}

bool checkFrontVisited() {
  return checkVisited(current_direction);
}

bool checkLeftVisited() {
  return checkVisited(updateValue(current_direction, 4, -1));
}

bool checkRightVisited() {
  return checkVisited(updateValue(current_direction, 4, 1));
}

void setEncoderCounts(int target = 100, int initial = 101) {
  target_encoder_count = target;
  left_encoder_count = initial;
  right_encoder_count = initial;
}

void updateEncoderCounts() {
  int left_encoder_value = digitalRead(LEFT_ENCODER_PIN);
  int right_encoder_value = digitalRead(RIGHT_ENCODER_PIN);
  if (left_encoder_value != left_encoder_last) left_encoder_count++;
  if (right_encoder_value != right_encoder_last) right_encoder_count++;
  left_encoder_last = left_encoder_value;
  right_encoder_last = right_encoder_value;
}

void stop(int x = 100) {
  LServo.writeMicroseconds(1500);
  RServo.writeMicroseconds(1500);
  delay(x);
}

void correctMotion() {
  readSensorValues();
  if (left_distance == right_distance) {
    LServo.writeMicroseconds(1540);
    RServo.writeMicroseconds(1461);
  }
  else if (left_distance < 6 || (right_distance >= 8 && right_distance < SENSOR_SENSITIVITY + 1)) {
    LServo.writeMicroseconds(1543);
    RServo.writeMicroseconds(1461);
  } else if (right_distance < 6 || (left_distance >= 8 && left_distance < SENSOR_SENSITIVITY + 1)) {
    // add more to right wheel
    LServo.writeMicroseconds(1540);
    RServo.writeMicroseconds(1458);
  } else {
     LServo.writeMicroseconds(1540);
     RServo.writeMicroseconds(1461);
  }
}

void moveForward() {
  looking_for_color = true;
  setEncoderCounts(144, 0);
  LServo.writeMicroseconds(1540);
  RServo.writeMicroseconds(1461);
  while(target_encoder_count > right_encoder_count && target_encoder_count > left_encoder_count) {
    readColors();
    updateEncoderCounts();
    correctMotion();
  }
  stop();
  setEncoderCounts();
}

void turnLeft() {
  current_direction = updateValue(current_direction, 4, -1);
  setEncoderCounts(29, 0);
  LServo.writeMicroseconds(1460);
  RServo.writeMicroseconds(1460);
  while(target_encoder_count > right_encoder_count && target_encoder_count > left_encoder_count) {
    updateEncoderCounts();
  }
  stop();
  setEncoderCounts();
}

void turnRight() {
  current_direction = updateValue(current_direction, 4, 1);
  setEncoderCounts(29, 0);
  LServo.writeMicroseconds(1540);
  RServo.writeMicroseconds(1540);
  while(target_encoder_count > right_encoder_count && target_encoder_count > left_encoder_count) {
    updateEncoderCounts();
  }
  stop();
  setEncoderCounts();
}

void runPathPlanning() {
  readAvgSensorValues();
  bool wallLeft = checkLeftWall();
  bool wallFront = checkFrontWall();
  bool wallRight = checkRightWall();
  printMaze();
  if (!wallFront && !checkFrontVisited()) {
    markBoard();
    moveForward();
  } else if (!wallLeft && !checkLeftVisited()) {
    turnLeft();
  } else if (!wallRight && !checkRightVisited()) {
    turnRight();
  } else if (!wallFront) {
    markBoard();
    moveForward();
  } else if (!wallLeft) {
    turnLeft();
  } else if (!wallRight) {
    turnRight();
  } else {
    turnRight();
    turnRight();
  }
  getVisitedCount();
}

void checkSides(int position) {
  int row = position / 4;
  int col = position % 4;
  int cost = brushfire_maze[row][col] + 1;
  // if there is no wall west
  if (!maze[row][col].west && !brushfire_maze[row][col - 1]) {
    brushfire_maze[row][col - 1] = cost;
    queue.push(row*4 + (col - 1));
  }
  if (!maze[row][col].north && !brushfire_maze[row - 1][col]) {
    brushfire_maze[row - 1][col] = cost;
    queue.push((row - 1) * 4 + col);
  }
  if (!maze[row][col].east && !brushfire_maze[row][col + 1]) {
    brushfire_maze[row][col + 1] = cost;
    queue.push(row * 4 + (col + 1));
  }
  if (!maze[row][col].south && !brushfire_maze[row + 1][col]) {
    brushfire_maze[row + 1][col] = cost;
    queue.push((row + 1) * 4 + col);
  }
}

void brushfire() {
  brushfire_maze[target_row][target_col] = 1;
  queue.push(target_row * 4 + target_col);
  while(!queue.isEmpty()) {
    int position = queue.pop();
    checkSides(position);
  }
}

int findSmallestNeighbor() {
  int target_cost = brushfire_maze[current_row][current_col] - 1;
  cell current_cell = maze[current_row][current_col];
  if (current_row != 0 && !maze[current_row][current_col].north && brushfire_maze[current_row - 1][current_col] == target_cost) {
    current_row -= 1;
    return NORTH;
  }
  if (current_col != 3 && !maze[current_row][current_col].east && brushfire_maze[current_row][current_col + 1] == target_cost) {
    current_col += 1;
    return EAST;
  }
  if (current_row != 3 && !maze[current_row][current_col].south && brushfire_maze[current_row + 1][current_col] == target_cost) {
    current_row += 1;
    return SOUTH;
  }
  if (current_col != 0 && !maze[current_row][current_col].west && brushfire_maze[current_row][current_col - 1] == target_cost) {
    current_col -= 1;
    return WEST;
  }
  return NONE;
}

void turnToNeighbor(int dir) {
  if (current_direction == dir) return;
  if (dir == NONE) {
    current_direction = NONE;
    robot_state = ERROR_STATE;
    return;
  }

  if (updateValue(current_direction, 4, 1) == dir) {
    turnRight();
  } else if (updateValue(current_direction, 4, -1) == dir) {
    turnLeft();
  } else {
    turnRight();
    turnRight();
  }
  readAvgSensorValues();
  printMaze();
}

void runShortestPath() {
  readAvgSensorValues();
  maze[current_row][current_col].state = V;
  printMaze();
  if (current_row != target_row || current_col != target_col) {
    turnToNeighbor(findSmallestNeighbor());
    if (current_direction != NONE) moveForward();
  } else {
    delay(5000);
    robot_state += 1;
  }
}
