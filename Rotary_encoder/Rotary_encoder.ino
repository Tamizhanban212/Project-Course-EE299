#define ENCA 2
#define ENCB 4

int pos = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() {
  // int a = digitalRead(ENCA);
  // int b = digitalRead(ENCB);
  // Serial.print(a*5);
  // Serial.print(" ");
  // Serial.print(b*5);
  // Serial.println();
  Serial.println(pos);
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b>0){
    pos++;
  }
  else{
    pos--;
  }
}

// volatile long temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder
    
// void setup() {
//   Serial.begin (9600);
//   pinMode(4, INPUT_PULLUP); // internal pullup input pin 2 
//   pinMode(2, INPUT_PULLUP); // internal pullup input pin 3
//    //Setting up interrupt
//   //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
//   attachInterrupt(0, ai0, RISING);
//   //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
//   attachInterrupt(1, ai1, RISING);
//   }

// void loop() {
// // Send the value of counter
// if( counter != temp ){
// Serial.println (counter);
// temp = counter;
// }
// }

// void ai0() {
// // ai0 is activated if DigitalPin nr 2 is goinqg from LOW to HIGH
// // Check pin 3 to determine the direction
// if(digitalRead(2)!=LOW) {
// counter++;
// }else{
// counter--;
// }
// }
  
// void ai1() {
// // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
// // Check with pin 2 to determine the direction
// if(digitalRead(4)!=LOW) {
// counter--;
// }else{
// counter++;
// }
// }