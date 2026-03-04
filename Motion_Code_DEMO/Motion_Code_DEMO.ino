#include <Servo.h>

Servo myservo1; //FRONT RIGHT
Servo myservo2; //BACK RIGHT
Servo myservo3; //BACK LEFT
Servo myservo4; //FRONT LEFT

char cmd = 'S';   // Default state = Stop

void setup() {
  Serial.begin(9600);

  myservo1.attach(3);
  myservo2.attach(2);
  myservo3.attach(A0); //Reversed
  myservo4.attach(12); //Reversed

  stop_motors();   // Stop at startup (IMPORTANT)
}

void loop() {

  // Read serial safely
  if (Serial.available() > 0) {
    char incoming = Serial.read();

    // Accept only valid commands
    if (incoming == 'F' || incoming == 'B' || 
        incoming == 'L' || incoming == 'R' || 
        incoming == 'S') {
      cmd = incoming;
    }
  }

  // Execute command
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
// MOVEMENT FUNCTIONS
// =======================

// FORWARD
void move_forward() {
  myservo1.write(180);
  myservo2.write(180);
  myservo3.write(0);
  myservo4.write(0);
}

// BACKWARD
void move_backward() {
  myservo1.write(0);
  myservo2.write(0);
  myservo3.write(180);
  myservo4.write(180);
}

// LEFT TURN
void turn_left() {
  myservo1.write(180);
  myservo2.write(180);
  myservo3.write(180);
  myservo4.write(180);
}

// RIGHT TURN
void turn_right() {
  myservo1.write(0);
  myservo2.write(0);
  myservo3.write(0);
  myservo4.write(0);
}

// STOP
void stop_motors() {
  myservo1.write(90);
  myservo2.write(90);
  myservo3.write(90);
  myservo4.write(90);
}
