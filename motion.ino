#include <Servo.h>

Servo myservo1; //FRONT RIGHT
Servo myservo2; //BACK RIGHT
Servo myservo3; //BACK LEFT
Servo myservo4; //FRONT LEFT

// Current speeds: 0-180 (90 = stop)
int right_speed = 90;  // Servos 1, 2
int left_speed = 90;   // Servos 3, 4
char cmd = 'S';        // Default state = Stop

// Function declarations
void parseCommand(String input);
void move_forward();
void move_backward();
void turn_left();
void turn_right();
void stop_motors();

void setup() {
  Serial.begin(115200);
  myservo1.attach(3);
  myservo2.attach(2);
  myservo3.attach(A0); //Reversed
  myservo4.attach(12); //Reversed
  stop_motors();
}

void loop() {
  // Read serial safely
  if (Serial.available() > 0) {
    String incoming = Serial.readStringUntil('\n');
    incoming.trim();
    
    // Parse command format: "F80" or "F80,F60" (right,left) or "S"
    parseCommand(incoming);
  }
  
  // Execute command with current speeds
  switch (cmd) {
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
// COMMAND PARSING
// =======================
void parseCommand(String input) {
  // Format: "F80" (symmetric), "F80,F60" (right,left), or "S" (stop)
  
  // Handle stop command (no number)
  if (input == "S") {
    cmd = 'S';
    return;
  }
  
  int commaIndex = input.indexOf(',');
  String rightCmd, leftCmd;
  
  if (commaIndex != -1) {
    // Two separate commands: "F80,F60"
    rightCmd = input.substring(0, commaIndex);
    leftCmd = input.substring(commaIndex + 1);
  } else {
    // Single command applies to both: "F80"
    rightCmd = input;
    leftCmd = input;
  }
  
  // Parse right motors (servo 1, 2)
  if (rightCmd.length() >= 2) {
    char cmdChar = rightCmd.charAt(0);
    int speed = rightCmd.substring(1).toInt();
    
    // Validate speed (0-100 = 0-180 range)
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;
    
    if (cmdChar == 'F' || cmdChar == 'B' || 
        cmdChar == 'L' || cmdChar == 'R') {
      cmd = cmdChar;
      right_speed = map(speed, 0, 100, 0, 180);
    }
  }
  
  // Parse left motors (servo 3, 4)
  if (leftCmd.length() >= 2) {
    char cmdChar = leftCmd.charAt(0);
    int speed = leftCmd.substring(1).toInt();
    
    // Validate speed (0-100 = 0-180 range)
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;
    
    if (cmdChar == 'F' || cmdChar == 'B' || 
        cmdChar == 'L' || cmdChar == 'R') {
      cmd = cmdChar;
      left_speed = map(speed, 0, 100, 0, 180);
    }
  }
}

// =======================
// MOVEMENT FUNCTIONS
// =======================
void move_forward() {
  myservo1.write(right_speed);
  myservo2.write(right_speed);
  myservo3.write(180 - left_speed);
  myservo4.write(180 - left_speed);
}

void move_backward() {
  myservo1.write(180 - right_speed);
  myservo2.write(180 - right_speed);
  myservo3.write(left_speed);
  myservo4.write(left_speed);
}

void turn_left() {
  // Right motors forward, left motors backward
  myservo1.write(right_speed);
  myservo2.write(right_speed);
  myservo3.write(left_speed);
  myservo4.write(left_speed);
}

void turn_right() {
  // Right motors backward, left motors forward
  myservo1.write(180 - right_speed);
  myservo2.write(180 - right_speed);
  myservo3.write(180 - left_speed);
  myservo4.write(180 - left_speed);
}

void stop_motors() {
  myservo1.write(90);
  myservo2.write(90);
  myservo3.write(90);
  myservo4.write(90);
  right_speed = 90;
  left_speed = 90;
}