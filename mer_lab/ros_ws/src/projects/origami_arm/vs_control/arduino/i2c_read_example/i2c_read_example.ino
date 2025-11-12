#include <Wire.h>

void receiveMotorCommand(int howMany);
//void requestEncoderData();
//void requestTouchState();


byte motorCommand[6];  // previous is 3
byte motor_state = 0;
int command_type;

int max_module_length;
int min_module_length;

int des_length1;
int des_length2;
int des_length3;

void setup()
{
  Wire.begin(11);
  //  Wire.onReceive(receiveEvent);
  Wire.onReceive(receiveMotorCommand);  // register event
  //  Wire.onRequest(requestEncoderData);

  Serial.begin(9600);
}


void loop()
{
  delay(100);
}

void receiveMotorCommand(int howMany) {

  int index = 0;
  while (Wire.available()) {
    byte d = Wire.read();
    if (index < 6) {
      motorCommand[index] = d;
    } else {
      command_type = d;
    }
    index = index + 1;

    Serial.print(d);
    Serial.print("\t");
  }

  if (command_type == 0){
    // for initial setting
    max_module_length = (motorCommand[0] << 8) + motorCommand[1];
    min_module_length = (motorCommand[2] << 8) + motorCommand[3];
    //des_length3 = (motorCommand[4] << 8) + motorCommand[5];
    Serial.print("cmd: init\n");
    Serial.print("max module length:\t");
    Serial.print(max_module_length);
    Serial.print("\t");
    Serial.print("min module length\t");
    Serial.print(min_module_length);
    Serial.print("\n");

  }
  else if(command_type == 1){
    // Need to make this 4 byte eventually
    des_length1 = (motorCommand[0] << 8) + motorCommand[1];
    des_length2 = (motorCommand[2] << 8) + motorCommand[3];
    des_length3 = (motorCommand[4] << 8) + motorCommand[5];

    Serial.print("cmd: position cmd\n");
    Serial.print("l1:\t");
    Serial.print(des_length1);
    Serial.print("\t");
    Serial.print("l2:\t");
    Serial.print(des_length2);
    Serial.print("\t");
    Serial.print("l3:\t");
    Serial.print(des_length3);
    Serial.print("\n");

  }
  else{
    Serial.print("cmd: velocity cmd\n");
    
  }
  
  // Example how to make 4 byte variable
  // des_length1 = ((long)(motorCommand[0])<<24) +
  // ((long)(motorCommand[1])<<16)
  // + ((long)(motorCommand[2])<<8) + ((long)(motorCommand[3])) ;  //
  // motor command in 4 bytes
}



void receiveEvent(int howMany)      
{
  while(1 < Wire.available())       
    {
      char c = Wire.read();          
      Serial.print(c);             
    }
  int x = Wire.read();
  Serial.println(x);
}
