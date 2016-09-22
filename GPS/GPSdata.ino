void setup()
{
  Serial.begin(4800);
}
void loop()
{
  while(1)
  {
  byte a;
  if ( Serial.available() > 0 )
  {
    a = Serial.read(); // get the byte of data from the GPS
    Serial.write(a);
  }
  }
}
