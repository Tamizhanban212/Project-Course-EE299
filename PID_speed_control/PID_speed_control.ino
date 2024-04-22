template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order+1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order+1]; // Raw values
    float y[order+1]; // Filtered values

  public:  
    LowPass(float f0, float fs, bool adaptive){
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.
      
      omega0 = 6.28318530718*f0;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){
        x[k] = 0;
        y[k] = 0;        
      }
      setCoef();
    }

    void setCoef(){
      if(adapt){
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }
      
      float alpha = omega0*dt;
      if(order==1){
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);        
      }
      if(order==2){
        float alphaSq = alpha*alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
        b[0] = alphaSq/D;
        b[1] = 2*b[0];
        b[2] = b[0];
        a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
        a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;      
      }
    }

    float filt(float xn){
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if(adapt){
        setCoef(); // Update coefficients if necessary      
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];
      }
      y[0] += b[order]*x[order];

      // Save the historical values
      for(int k = order; k > 0; k--){
        y[k] = y[k-1];
        x[k] = x[k-1];
      }
  
      // Return the filtered value    
      return y[0];
    }
};

// Filter instance
LowPass<2> lp(3,1e3,true);

#include <util/atomic.h>

#define ENCA 2
#define ENCB 4
#define Dir 7
#define PWM 8

// globals
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
  setMotor(0, 0, PWM, Dir);
  delay(2000);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() 
{
  // int pwr = 200/5.0*micros()/1.0e6;
  // int dir = 1;
  // setMotor(dir, pwr, PWM, Dir);

  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    // velocity2 = velocity_i;
  }

  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;
  float v1 = velocity1/28754.0*60.0;
  v1Filt = lp.filt(v1);
  
  // Set target speed
  float vt = 100;

  // control signal equation u
  float kp = 0.7;
  float ki = 0.04;
  float e = vt-v1Filt;
  eintegral = eintegral + e*deltaT;
  
  float u = kp*e + ki*eintegral;
  
  int dir = 1;
  if (u<0){
    dir = 0;
  }

  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  setMotor(dir, pwr, PWM, Dir);

  Serial.print("Target:");
  Serial.print(vt);
  Serial.print(" ");
  Serial.print("MotorVel:");
  Serial.print(v1Filt);
  Serial.println();
  delay(1);
}

void setMotor(int DirIn, int PWMval, int pwm, int dir){
  analogWrite(pwm,PWMval);
  digitalWrite(dir,DirIn);
}

void readEncoder(){
  int b = digitalRead(ENCB);
  int increment = 0;
  if (b>0){
    increment = -1; // since ENCB would have already be triggered on clockwise direction
  }
  else{
    increment = 1;
  }
  pos_i = pos_i + increment;

  // long currT = micros();
  // float deltaT = ((float)(currT - prevT_i))/1.0e6;
  // velocity_i = increment/deltaT;
  // prevT_i = currT;
}

// long prevT = 0;
// int posPrev = 0;
// volatile int pos_i = 0;
// volatile float velocity_i = 0;
// volatile long prevT_i = 0;

// float eintegral = 0;

// float v1Filt = 0;
// void setup() {
//   Serial.begin(9600);
//   pinMode(ENCA,INPUT);
//   pinMode(ENCB,INPUT);
//   pinMode(Dir, OUTPUT);
//   pinMode(PWM, OUTPUT);
//   attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
// }

// void loop() {
//   int pos = 0;
//   // float velocity2 = 0;
//   ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
//     pos = pos_i;
//     // velocity2 = velocity_i;
//   }

//   long currT = micros();
//   float deltaT = ((float)(currT-prevT))/1.0e6;
//   float velocity1 = (pos - posPrev)/deltaT;
//   posPrev = pos;
//   prevT = currT;

//   float v1 = velocity1/84227.0*60.0;
//   // float v2 = velocity2/84227.0*60.0;

//   // v1Filt = lp.filt(v1);

//   float vt = 9;//9*(sin(currT/1e6)>0);

//   float kp = 10;
//   float ki = 0;
//   float e = vt-v1;
//   eintegral = eintegral + e*deltaT;

//   float u = kp*e + ki*eintegral;

//   int dir = 1;
//   if (u<0){
//     dir = 0;
//   }
//   int pwr = (int) fabs(u);
//   if (pwr>255){
//     pwr = 255;
//   }

//   setMotor(dir,pwr,PWM,Dir);

//   Serial.print("Velocity:");
//   Serial.print(v1);
//   Serial.print(" ");
//   Serial.print("VelocitySet:");
//   Serial.print(vt);
//   Serial.println();
//   delayMicroseconds(200);
// }

// void readEncoder(){
//   int b = digitalRead(ENCB);
//   int increment = 0;
//   if (b>0){
//     increment = -1; // since ENCB would have already be triggered on clockwise direction
//   }
//   else{
//     increment = +1;
//   }
//   pos_i = pos_i + increment;

//   // long currT = micros();
//   // float deltaT = ((float)(currT - prevT_i))/1.0e6;
//   // velocity_i = increment/deltaT;
//   // prevT_i = currT;
// }

// void setMotor(int DirIn, int PWMval, int pwm, int dir){
//   analogWrite(pwm,PWMval);
//   digitalWrite(dir,DirIn);
// }