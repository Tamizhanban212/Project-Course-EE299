#define MotorDirection 7
#define MotorSpeed 8

int SpeedVal = 0;
int dir = 1;
int delayTime = 2000; // delay time in milliseconds

void setup() {
  pinMode(MotorDirection, OUTPUT);
  pinMode(MotorSpeed, OUTPUT);

  analogWrite(MotorSpeed, 0);
  delay(delayTime);
}

void loop() {
  // Rotate the motor clockwise
  dir = 1;
  digitalWrite(MotorDirection, dir);
  analogWrite(MotorSpeed, SpeedVal);
}
