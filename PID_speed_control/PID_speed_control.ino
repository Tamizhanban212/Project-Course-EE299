#include <util/atomic.h>

#define ENCA 2
#define ENCB 4
#define Dir 7
#define PWM 8

long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float eintegral = 0;

float v1Filt = 0;
void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(Dir, OUTPUT);
  pinMode(PWM, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
  int pos = 0;
  // float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    // velocity2 = velocity_i;
  }

  long currT = micros();
  float deltaT = ((float)(currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  float v1 = velocity1/84227.0*60.0;
  // float v2 = velocity2/84227.0*60.0;

  // v1Filt = lp.filt(v1);

  float vt = 9*(sin(currT/1e6)>0);

  float kp = 0.001;
  float ki = 1000;
  float e = vt-v1;
  eintegral = eintegral + e*deltaT;

  float u = kp*e + ki*eintegral;

  int dir = 1;
  if (u<0){
    dir = 0;
  }
  int pwr = (int) fabs(u);
  if (pwr>255){
    pwr = 255;
  }

  setMotor(dir,pwr,PWM,Dir);

  Serial.print("Velocity:");
  Serial.print(v1);
  Serial.print(" ");
  Serial.print("VelocitySet:");
  Serial.print(vt);
  Serial.println();
  delayMicroseconds(200);
}

void readEncoder(){
  int b = digitalRead(ENCB);
  int increment = 0;
  if (b>0){
    increment = -1; // since ENCB would have already be triggered on clockwise direction
  }
  else{
    increment = +1;
  }
  pos_i = pos_i + increment;

  // long currT = micros();
  // float deltaT = ((float)(currT - prevT_i))/1.0e6;
  // velocity_i = increment/deltaT;
  // prevT_i = currT;
}

void setMotor(int DirIn, int PWMval, int pwm, int dir){
  analogWrite(pwm,PWMval);
  digitalWrite(dir,DirIn);
}