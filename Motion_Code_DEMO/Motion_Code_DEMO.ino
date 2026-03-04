
#include <Servo.h>

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;

char cmd = 'S';

void setup() {
  Serial.begin(9600);

  myservo1.attach(9);
  myservo2.attach(10);
  myservo3.attach(11);
  myservo4.attach(12);

  stop_motors();
}

void loop() {

  if (Serial.available() > 0) {
    cmd = Serial.read();
  }

  if (cmd == 'F') {
    move_forward();
  }
  else if (cmd == 'B') {
    move_backward();
  }
  else if (cmd == 'L') {
    turn_left();
  }
  else if (cmd == 'R') {
    turn_right();
  }
  else if (cmd == 'S') {
    stop_motors();
  }
}


// FORWARD (as you defined)
void move_forward() {
  myservo1.write(0);
  myservo2.write(180);
  myservo3.write(0);
  myservo4.write(180);
}

// BACKWARD = opposite of forward
void move_backward() {
  myservo1.write(180);
  myservo2.write(0);
  myservo3.write(180);
  myservo4.write(0);
}

// LEFT turn
void turn_left() {
  myservo1.write(180);   // left side backward
  myservo2.write(180);
  myservo3.write(0);     // right side forward
  myservo4.write(0);
}

// RIGHT turn
void turn_right() {
  myservo1.write(0);     // left side forward
  myservo2.write(0);
  myservo3.write(180);   // right side backward
  myservo4.write(180);
}

// STOP (neutral)
void stop_motors() {
  myservo1.write(90);
  myservo2.write(90);
  myservo3.write(90);
  myservo4.write(90);
}
