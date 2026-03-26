#include <Servo.h>

Servo myservo1; //FRONT RIGHT
Servo myservo2; //BACK RIGHT
Servo myservo3; //BACK LEFT
Servo myservo4; //FRONT LEFT

char direction = 'S';
int speedValue = 30;

void setup() {

  Serial.begin(115200);

  myservo1.attach(3);
  myservo2.attach(2);
  myservo3.attach(A0);
  myservo4.attach(12);

  stop_motors();

  Serial.println("Send commands like F40 B30 L50 R60");
}

void loop() {

  if (Serial.available()) {

    String cmd = Serial.readStringUntil('\n');

    direction = cmd.charAt(0);         // First letter
    speedValue = cmd.substring(1).toInt(); // Number after letter

    Serial.print("Direction: ");
    Serial.println(direction);

    Serial.print("Speed: ");
    Serial.println(speedValue);
  }

  switch(direction) {

    case 'F':
      move_forward(speedValue);
      break;

    case 'B':
      move_backward(speedValue);
      break;

    case 'L':
      turn_left(speedValue);
      break;

    case 'R':
      turn_right(speedValue);
      break;

    case 'S':
      stop_motors();
      break;
  }
}

void move_forward(int spd) {
  myservo1.write(90 + spd);
  myservo2.write(90 + spd);
  myservo3.write(90 - spd);
  myservo4.write(90 - spd);
}

void move_backward(int spd) {

  myservo1.write(90 - spd);
  myservo2.write(90 - spd);
  myservo3.write(90 + spd);
  myservo4.write(90 + spd);
}

void turn_left(int spd) {

  myservo1.write(90 + spd);
  myservo2.write(90 + spd);
  myservo3.write(90 + spd);
  myservo4.write(90 + spd);
}

void turn_right(int spd) {

  myservo1.write(90 - spd);
  myservo2.write(90 - spd);
  myservo3.write(90 - spd);
  myservo4.write(90 - spd);
}

void stop_motors() {
  myservo1.write(90);
  myservo2.write(90);
  myservo3.write(90);
  myservo4.write(90);
}