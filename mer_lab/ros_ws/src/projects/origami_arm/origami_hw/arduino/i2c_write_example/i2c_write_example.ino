  #include <Wire.h>
byte x = 0;                        

char cmd[7];

void setup()
{
  Wire.begin();
  for (int i=0; i<6; i++){
    cmd[i] = 2;
  }
  cmd[6] = 2;
}

void loop()
{
  Wire.beginTransmission(11);
  Wire.write(cmd,sizeof(cmd));
  //  Wire.write(x);
  Wire.endTransmission();

  //  x++;
  delay(500);
}
