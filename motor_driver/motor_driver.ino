/*Example code for testing the Cytron Technologies 10A Rev 2.0 Motor Driver
 *The code uses only two pins, one for the motor direction (Pin 9) 
 * and one for the motor speed (pin 9). The speed is set using PWM. 
 * This code is meant to be run on the Arduino Uno hardware. 
 */


//Pins used to control direction and speed of the motor. Speed pin should be a pwm pin.
#define MotorDirection 7
#define MotorSpeed 8

int SpeedVal = 0;

void setup() {
  //Declaration for the pins used, both should be outputs.
  pinMode(MotorDirection, OUTPUT);
  pinMode(MotorSpeed, OUTPUT);
}

void loop() {
  
  //Ramps up the speed in the clockwise direction.
  digitalWrite(MotorDirection, 1);                  //Loop increases the speed slowly until it reaches max speed.
  analogWrite(MotorSpeed,SpeedVal);

}

