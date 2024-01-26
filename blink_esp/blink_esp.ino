void setup()
{
  pinMode(D4, OUTPUT);
}

void loop()
{
  digitalWrite(D4, HIGH);
  delay(50);
  digitalWrite(D4, LOW);
  delay(50);
}