// // Code for reading encoder output

// #define ENCA 2
// #define ENCB 4

// void setup() 
// {
//   Serial.begin(9600);
//   pinMode(ENCA, INPUT);
//   pinMode(ENCB, INPUT);

// }

// void loop() 
// {
//   int a = digitalRead(ENCA);
//   int b = digitalRead(ENCB);
//   Serial.print(a*5);
//   Serial.print(" ");
//   Serial.print(b*5);
//   Serial.println();
// }

// // Measure position using encoder

// #define ENCA 2
// #define ENCB 4

// int pos = 0;

// void setup() 
// {
//   Serial.begin(9600);
//   pinMode(ENCA, INPUT);
//   pinMode(ENCB, INPUT);
//   attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
// }

// void loop() 
// {
//   Serial.println(pos);
// }

// void readEncoder()
// {
//   int b = digitalRead(ENCB);
//   if(b>0)
//   {
//     pos++;
//   }
//   else
//   {
//     pos--;
//   }
// }

// 

#define ENCA 2
#define ENCB 4
#define Dir 7
#define PWM 8

int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() 
{
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  setMotor(0,0,PWM,Dir);
  delay(2000);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() 
{
  // set target position
  int target = 7000;
  
  // set PID constants
  float kp = 0.099;
  float kd = 0;
  float ki = 0.0001;

  // time difference
  long currT = micros();

  float deltaT = ((float)(currT-prevT))/1.0e6;
  prevT = currT;

  //error
  int e = pos - target;
  
  // derivative
  float dedt = (e-prevT)/(deltaT);

  // integral
  eintegral = eintegral - e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if ((pwr>30) && (e<1000))
  {
    pwr = 35;
  }

  // motor direction
  int dir = 1;
  if(u<0)
  {
    dir = 0;
  }

  // signal the motor
  setMotor(dir, pwr, PWM, Dir);

  // store the previous error
  eprev = 0;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();

  // setMotor(1,50,PWM,Dir);
  // delay(200);
  // Serial.println(pos);
  // setMotor(0,50,PWM,Dir);
  // delay(200);
  // Serial.println(pos);
}

void readEncoder()
{
  int b = digitalRead(ENCB);
  if(b>0)
  {
    pos++;
  }
  else
  {
    pos--;
  }
}

void setMotor(int DirIn, int PWMval, int pwm, int dir){
  analogWrite(pwm,PWMval);
  digitalWrite(dir,DirIn);
}