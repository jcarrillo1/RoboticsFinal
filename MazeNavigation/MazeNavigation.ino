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
#define SENSOR_SENSITIVITY 10
#define LEFT_SPEED 1570
#define RIGHT_SPEED 1429

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
Servo LServo, RServo;

enum DIRECTIONS { NORTH, EAST, SOUTH, WEST, NONE };
enum FLOW_STATE { SETUP_ONE, PATH_PLANNING, FINISH_STATE, ERROR_STATE };
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
int color_target_count = 0;

struct cell {
  int state;
  bool north, south, west, east;
};

cell maze[4][4] = {
  {
    { 0, W, N, W, N },
    { 0, W, N, N, N },
    { 0, W, N, N, N },
    { 0, W, N, N, W }
  }, {
    { 0, N, N, W, N },
    { 0, N, N, N, N },
    { 0, N, N, N, N },
    { 0, N, N, N, W }
  }, {
    { 0, N, N, W, N },
    { 0, N, N, N, N },
    { 0, N, N, N, N },
    { 0, N, N, N, W }
  }, {
    { 0, N, W, W, N },
    { 0, N, W, N, N },
    { 0, N, W, N, N },
    { 0, N, W, N, W }
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
    case FINISH_STATE:
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
  maze[current_row][current_col].state = 1;
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

  if (looking_for_color && (fRed < 150 || fBlue < 150)) {
    updateLcdColor();
    looking_for_color = false;
    color_target_count = 75;
    left_encoder_count = 67;
    right_encoder_count = 67;
  } else if (!looking_for_color && (color_target_count <= 0 || target_encoder_count == 0)) {
    lcd.setBacklight(WHITE);
  } else {
    color_target_count -= 1;
  }
}

void markBoard() {
  if (current_direction == NORTH) current_row -= 1;
  if (current_direction == EAST) current_col += 1;
  if (current_direction == SOUTH) current_row += 1;
  if (current_direction == WEST) current_col -= 1;
  maze[current_row][current_col].state += 1;
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
    robot_state += 1;
  }
}

bool checkWall(int direction_to_check, int distance_to_check) {
  bool isWall = distance_to_check < SENSOR_SENSITIVITY && distance_to_check > 0;
  switch(direction_to_check) {
    case NORTH:
      if (current_row == 0 || maze[current_row][current_col].north) return true;
      if (isWall && !maze[current_row][current_col].north) {
        maze[current_row][current_col].north = W;
        maze[current_row - 1][current_col].south = W;
      }
      break;
    case EAST:
      if (current_col == 3 || maze[current_row][current_col].east) return true;
      if (isWall && !maze[current_row][current_col].east) {
        maze[current_row][current_col].east = W;
        maze[current_row][current_col + 1].west = W;
      }
      break;
    case SOUTH:
      if (current_row == 3 || maze[current_row][current_col].south) return true;
      if (isWall && !maze[current_row][current_col].south) {
        maze[current_row][current_col].south = W;
        maze[current_row + 1][current_col].north = W;
      }
      break;
    case WEST:
      if (current_col == 0 || maze[current_row][current_col].west) return true;
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

int checkVisited(int direction_to_check) {
  switch(direction_to_check) {
    case NORTH:
      if (current_row != 0) return maze[current_row - 1][current_col].state;
    case EAST:
      if (current_col != 3) return maze[current_row][current_col + 1].state;
    case SOUTH:
      if (current_row != 3) return maze[current_row + 1][current_col].state;
    case WEST:
      if (current_col != 0) return maze[current_row][current_col - 1].state;
    default:
      return 10;
  }
}

int checkFrontVisited() {
  return checkVisited(current_direction);
}

int checkLeftVisited() {
  return checkVisited(updateValue(current_direction, 4, -1));
}

int checkRightVisited() {
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
  if (right_distance > SENSOR_SENSITIVITY && left_distance > SENSOR_SENSITIVITY) {
    LServo.writeMicroseconds(LEFT_SPEED);
    RServo.writeMicroseconds(RIGHT_SPEED);
  } else if (left_distance > 10 || left_distance < 1) {
    if (right_distance > 7) {
      LServo.writeMicroseconds(LEFT_SPEED + 40);
      RServo.writeMicroseconds(RIGHT_SPEED);

    } else if (right_distance < 7) {
      LServo.writeMicroseconds(LEFT_SPEED);
      RServo.writeMicroseconds(RIGHT_SPEED - 40);
    } else {
      LServo.writeMicroseconds(LEFT_SPEED);
      RServo.writeMicroseconds(RIGHT_SPEED);
    }
  } else {
    if (left_distance > 7) {
      LServo.writeMicroseconds(LEFT_SPEED);
      RServo.writeMicroseconds(RIGHT_SPEED - 40);
    } else if (left_distance < 7) {
      LServo.writeMicroseconds(LEFT_SPEED + 40);
      RServo.writeMicroseconds(RIGHT_SPEED);
    } else {
      LServo.writeMicroseconds(LEFT_SPEED);
      RServo.writeMicroseconds(RIGHT_SPEED);
    }
  }
}

void moveForward() {
  looking_for_color = true;
  setEncoderCounts(138, 0);
  LServo.writeMicroseconds(LEFT_SPEED);
  RServo.writeMicroseconds(RIGHT_SPEED);
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
  int frontVisited = checkFrontVisited();
  int leftVisited = checkLeftVisited();
  int rightVisited = checkRightVisited();
  printMaze();
  if (!wallFront && !frontVisited) {
    markBoard();
    moveForward();
  } else if (!wallLeft && !leftVisited) {
    turnLeft();
  } else if (!wallRight && !rightVisited) {
    turnRight();
  } else if (!wallFront && (wallLeft || frontVisited <= leftVisited) && (wallRight || frontVisited <= rightVisited)) {
    markBoard();
    moveForward();
  } else if (!wallLeft && (wallFront || leftVisited <= frontVisited) && (wallRight || leftVisited <= rightVisited)) {
    turnLeft();
  } else if (!wallRight && (wallFront || rightVisited <= frontVisited) && (wallLeft || rightVisited <= leftVisited)) {
    turnRight();
  } else if (!wallFront) {
    markBoard();
    moveForward();
  } else if (!wallLeft) {
    turnLeft();
  } else if (!wallRight) {
    turnRight();
  }else {
    turnRight();
    turnRight();
  }
  getVisitedCount();
}
