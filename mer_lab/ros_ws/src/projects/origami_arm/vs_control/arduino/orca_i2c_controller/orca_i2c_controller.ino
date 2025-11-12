/*
 * rosserial sub and send msg out via i2c
 */

#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <Wire.h>


ros::NodeHandle  nh;

const int i2cTargetId = 11;

void velCmdCb(std_msgs::UInt8MultiArray& msg){
  byte cmd[7];
  for (int i=0; i<7; i++){
    cmd[i] = msg.data[i];
  }
  Wire.beginTransmission(i2cTargetId);
  Wire.write(cmd, 7); // cmd length is 7
  Wire.endTransmission();

}

ros::Subscriber<std_msgs::UInt8MultiArray> sub1("origami_vs/velocity",velCmdCb);

void setup()
{
  Wire.begin();
  
  nh.initNode();
  nh.subscribe(sub1);
}

void loop()
{
  nh.spinOnce();
  //delay(500);
}
