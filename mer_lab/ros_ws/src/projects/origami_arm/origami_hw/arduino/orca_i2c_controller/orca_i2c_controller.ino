/*
 * rosserial sub and send msg out via i2c
 */

#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <Wire.h>


ros::NodeHandle  nh;

const int i2cTargetId = 11; //0x0B

// const int i2cTargetId1 = 10; // 0x0A
const int i2cTargetId2 = 12; // 0x0C

void velCmdCb1(std_msgs::UInt8MultiArray& msg){
  byte cmd[7];
  for (int i=0; i<7; i++){
    cmd[i] = msg.data[i];
  }
  Wire.beginTransmission(i2cTargetId1);
  Wire.write(cmd, 7); // cmd length is 7
  Wire.endTransmission();

}

void velCmdCb2(std_msgs::UInt8MultiArray& msg){
  byte cmd[7];
  for (int i=0; i<7; i++){
    cmd[i] = msg.data[i];
  }
  Wire.beginTransmission(i2cTargetId2);
  Wire.write(cmd, 7); // cmd length is 7
  Wire.endTransmission();

}

ros::Subscriber<std_msgs::UInt8MultiArray> sub1("origami_vs/velocity1",velCmdCb1);
ros::Subscriber<std_msgs::UInt8MultiArray> sub2("origami_vs/velocity2", velCmdCb2);

void setup()
{
  Wire.begin();
  
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
}

void loop()
{
  nh.spinOnce();
  //delay(500);
}
