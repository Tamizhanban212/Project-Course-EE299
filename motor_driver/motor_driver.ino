#define MotorDirection 7
#define MotorSpeed 8

int SpeedVal = 0;
int dir = 0;

void setup() {
  pinMode(MotorDirection, OUTPUT);
  pinMode(MotorSpeed, OUTPUT);

  analogWrite(MotorSpeed, 0);
  delay(2000);
}

void loop() {
  // Keep the motor running at the desired speed
  digitalWrite(MotorDirection, dir);
  analogWrite(MotorSpeed, SpeedVal);
}
