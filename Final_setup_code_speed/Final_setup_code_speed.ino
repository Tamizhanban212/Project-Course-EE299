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

#define ENCA 2
#define ENCB 4
#define Dir 7
#define PWM 8

// for trapezoidal velocity profile
// volatile float vt = 0; // Target velocity
// volatile float kp = 1.05; // Proportional gain
// volatile float ki = 0.00005; // Integral gain
// volatile float kd = 0.0001; // Derivative gain

// for rectangular velocity profile
// volatile float vt = 0; // Target velocity
// volatile float kp = 1.05; // Proportional gain
// volatile float ki = 0.00021; // Integral gain
// volatile float kd = 0.0013125; // Derivative gain

// for s-curve velocity profile
volatile float vt = 0; // Target velocity
volatile float kp = 1.05; // Proportional gain
volatile float ki = 0.00006; // Integral gain
volatile float kd = 0.0001; // Derivative gain

volatile long lastUpdateTime = 0;
volatile long encoderCount = 0; // Encoder count
volatile long lastEncoderCount = 0; // Last encoder count
volatile float velocity = 0.0; // Current velocity
float v1Filt = 0;

void setMotor(int DirIn, int PWMval, int pwm, int dir){
  analogWrite(pwm, PWMval);
  digitalWrite(dir,DirIn);
}

void setup() {
  Serial.begin(9600); // Start the serial communication with the baud rate of 9600
  setMotor(0, 0, PWM, Dir);
  delay(2000);
  attachInterrupt(digitalPinToInterrupt(ENCA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB), updateEncoder, CHANGE);
}

void loop()
{
  long elapsedTime = millis() % 10000; // time in the current cycle

  // trapezoidal velocity profile

  // if (elapsedTime < 1000) { // for the first 1 second
  //   vt = map(elapsedTime, 0, 1000, 0, 50); // linearly increase power from 0 to 50
  // } else if (elapsedTime < 3000) { // for the next 2 seconds
  //   vt = 50;
  // } else if (elapsedTime < 4000) { // for the next 1 second
  //   vt = map(elapsedTime, 3000, 4000, 50, 0); // linearly decrease power from 50 to 0
  // } else if (elapsedTime < 6000) { // for the next 2 seconds
  //   vt = 0;
  // } else if (elapsedTime < 7000) { // for the next 1 second
  //   vt = map(elapsedTime, 6000, 7000, 0, -50); // linearly increase power from 0 to -50 in reverse direction
  // } else if (elapsedTime < 9000) { // for the next 2 seconds
  //   vt = -50;
  // } else if (elapsedTime < 10000) { // for the last 1 second
  //   vt = map(elapsedTime, 9000, 10000, -50, 0); // linearly decrease power from -50 to 0 in reverse direction
  // }
  
  // rectangular velocity profile

  // if (elapsedTime < 2500) { // for the first 2.5 seconds
  //   vt = 50;
  // } else if (elapsedTime < 5000) { // for the next 2.5 seconds
  //   vt = 0;
  // } else if (elapsedTime < 7500) { // for the next 2.5 seconds
  //   vt = -50;
  // } else { // for the last 2.5 seconds
  //   vt = 0;
  // }

  // s-curve velocity profile

  // Define the constants for the logistic function
  float a1 = 10, c1 = 0.5, a2 = 10, c2 = 0.5;

  if (elapsedTime < 1000) { // for the first 1 second
    float x = elapsedTime / 1000.0; // normalize time to [0, 1]
    vt = 50 / (1 + exp(-a1 * (x - c1))); // logistic function for S-curve
  } else if (elapsedTime < 3000) { // for the next 2 seconds
    vt = 50;
  } else if (elapsedTime < 4000) { // for the next 1 second
    float x = (elapsedTime - 3000) / 1000.0; // normalize time to [0, 1]
    vt = 50 - 50 / (1 + exp(-a2 * (x - c2))); // logistic function for S-curve
  } else if (elapsedTime < 6000) { // for the next 2 seconds
    vt = 0;
  } else if (elapsedTime < 7000) { // for the next 1 second
    float x = (elapsedTime - 6000) / 1000.0; // normalize time to [0, 1]
    vt = -50 / (1 + exp(-a1 * (x - c1))); // logistic function for S-curve
  } else if (elapsedTime < 9000) { // for the next 2 seconds
    vt = -50;
  } else { // for the last 1 second
    float x = (elapsedTime - 9000) / 1000.0; // normalize time to [0, 1]
    vt = -50 + 50 / (1 + exp(-a2 * (x - c2))); // logistic function for S-curve
  }

  // Calculate the displacement
  long displacement = encoderCount - lastEncoderCount;
  lastEncoderCount = encoderCount;

  // Calculate the velocity
  velocity = ((float)displacement / 28754.0 * 60.0); // Counts per rotation is 28754
  v1Filt = lp.filt(velocity);

  // Calculate the PID control
  static float integral = 0;
  float error = vt - v1Filt;
  volatile float lastError = 0;

  integral += error;
  float derivative = (error - lastError);

  float output = kp * error + ki * integral + kd * derivative;
  lastError = error;

  // Set the motor speed
  setMotor(output >= 0 ? HIGH : LOW, min(abs(output), 255), PWM, Dir);

  // Send the set velocity and the actual velocity to the Serial Plotter
  // Send the set velocity and the actual velocity to the Serial Plotter
  Serial.print("Target:");
  Serial.print(vt);
  Serial.print(" ");
  Serial.print("MotorVel:");
  Serial.print(output);
  Serial.println();
  delay(1);
}

void updateEncoder() {
  // Update the encoder count
  if (digitalRead(ENCA) == digitalRead(ENCB)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}
