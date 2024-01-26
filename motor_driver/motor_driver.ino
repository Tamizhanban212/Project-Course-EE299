int a = 8;
int b = 9;

void setup() 
{
pinMode(8, OUTPUT);
pinMode(9, OUTPUT);
}

void loop() 
{
digitalWrite(a, LOW);
digitalWrite(b, HIGH);

}