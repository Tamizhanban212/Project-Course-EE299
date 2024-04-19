#define ENCA 2
#define ENCB 4
#define Dir 7
#define PWM 8

int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(Dir, OUTPUT);
  pinMode(PWM, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
  // target position
  int target = 3000*sin(prevT/1e6);
  // PID constants
  float kp = 1;
  float kd = 0;
  float ki = 10;

  long currT = micros();

  float deltaT = ((float)(currT-prevT))/1.0e6;
  prevT = currT;

  int e = target-pos;
  float dedt = (e-eprev)/(deltaT);
  eintegral = eintegral + e*deltaT;

  float u = kp*e + kd*dedt + ki*eintegral;
  
  float pwr = fabs(u);
  if (pwr>255){
    pwr = 255;
  }

  int dir = 1;
  if(u<0){
    dir = 0;
  }

  setMotor(dir,pwr,PWM,Dir);

  eprev = e; 

  Serial.print("Target:");
  Serial.print(target);
  Serial.print(" ");
  Serial.print("MotorPos:");
  Serial.print(pos);
  Serial.println();
  // setMotor(0,255,PWM,1);
  // delay(2000);
  // Serial.println(pos);
  // setMotor(1,255,PWM,Dir);
  // delay(2000);
  // Serial.println(pos);
  // setMotor(0,255,PWM,Dir);
  // delay(2000);
  // Serial.println(pos);
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if (b>0){
    pos++;
  }
  else{
    pos--;
  }
}

void setMotor(int DirIn, int PWMval, int pwm, int dir){
  analogWrite(pwm,PWMval);
  digitalWrite(dir,DirIn);
}