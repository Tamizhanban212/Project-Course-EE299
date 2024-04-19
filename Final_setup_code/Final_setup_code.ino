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

int pos = 0;

void setup() 
{
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() 
{
  Serial.println(pos);
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