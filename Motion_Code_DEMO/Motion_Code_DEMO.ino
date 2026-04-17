#include <Servo.h>

Servo myservo1; // FRONT RIGHT
Servo myservo2; // BACK RIGHT
Servo myservo3; // BACK LEFT
Servo myservo4; // FRONT LEFT

// Neutral
const int STOP = 90;

// Current speeds
int right_speed = STOP;
int left_speed = STOP;

char cmd = 'S';

// Trim (adjust manually if drifting)
int right_trim = 0;
int left_trim = 0;

// =======================
// SPEED MAPPING
// =======================

// Forward: 90 → 180 (right), 90 → 0 (left)
int mapForwardRight(int speed)
{
  return map(speed, 0, 100, 90, 180);
}

int mapForwardLeft(int speed)
{
  return map(speed, 0, 100, 90, 0);
}

// Backward: reverse directions
int mapBackwardRight(int speed)
{
  return map(speed, 0, 100, 90, 0);
}

int mapBackwardLeft(int speed)
{
  return map(speed, 0, 100, 90, 180);
}

// =======================
// SETUP
// =======================
void setup()
{

  Serial.begin(115200);
  myservo1.attach(3);
  myservo2.attach(2);
  myservo3.attach(A0); // reversed

  myservo4.attach(12); // reversed

  stop_motors();
}

// =======================
// LOOP
// =======================
void loop()
{

  if (Serial.available() > 0)
  {
    String incoming = Serial.readStringUntil('\n');
    incoming.trim();
    parseCommand(incoming);
  }

  switch (cmd)
  {
  case 'F':
    move_forward();
    break;
  case 'B':
    move_backward();
    break;
  case 'L':
    turn_left();
    break;
  case 'R':
    turn_right();
    break;
  case 'S':
    stop_motors();
    break;
  }
}

// =======================
// COMMAND PARSER
// =======================
void parseCommand(String input)
{

  if (input == "S")
  {
    cmd = 'S';
    return;
  }

  int commaIndex = input.indexOf(',');
  String rightCmd, leftCmd;

  if (commaIndex != -1)
  {
    rightCmd = input.substring(0, commaIndex);
    leftCmd = input.substring(commaIndex + 1);
  }
  else
  {
    rightCmd = input;
    leftCmd = input;
  }

  // RIGHT SIDE
  if (rightCmd.length() >= 2)
  {
    char c = rightCmd.charAt(0);
    int speed = rightCmd.substring(1).toInt();

    speed = constrain(speed, 0, 100);

    cmd = c;

    if (c == 'F')
      right_speed = mapForwardRight(speed);
    if (c == 'B')
      right_speed = mapBackwardRight(speed);
  }

  // LEFT SIDE
  if (leftCmd.length() >= 2)
  {
    char c = leftCmd.charAt(0);
    int speed = leftCmd.substring(1).toInt();

    speed = constrain(speed, 0, 100);

    if (c == 'F')
      left_speed = mapForwardLeft(speed);
    if (c == 'B')
      left_speed = mapBackwardLeft(speed);
  }
}

// =======================
// MOVEMENT
// =======================

void move_forward()
{
  myservo1.write(right_speed + right_trim);
  myservo2.write(right_speed + right_trim);
  myservo3.write(left_speed + left_trim);
  myservo4.write(left_speed + left_trim);
}

void move_backward()
{
  myservo1.write(right_speed + right_trim);
  myservo2.write(right_speed + right_trim);
  myservo3.write(left_speed + left_trim);
  myservo4.write(left_speed + left_trim);
}

void turn_left()
{
  // Right forward, left backward
  myservo1.write(mapForwardRight(60));
  myservo2.write(mapForwardRight(60));
  myservo3.write(mapBackwardLeft(60));
  myservo4.write(mapBackwardLeft(60));
}

void turn_right()
{
  // Right backward, left forward
  myservo1.write(mapBackwardRight(60));
  myservo2.write(mapBackwardRight(60));
  myservo3.write(mapForwardLeft(60));
  myservo4.write(mapForwardLeft(60));
}

void stop_motors()
{
  myservo1.write(STOP);
  myservo2.write(STOP);
  myservo3.write(STOP);
  myservo4.write(STOP);

  right_speed = STOP;
  left_speed = STOP;
}