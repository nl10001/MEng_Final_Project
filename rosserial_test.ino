#include <ros.h>
#include <Arduino.h>
#include <Braccio.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt8MultiArray.h>
#include <Servo.h>

ros::NodeHandle  nh;

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;

unsigned int _baseAngle = 90;
unsigned int _shoulderAngle = 90;
unsigned int _elbowAngle = 90;
unsigned int _wrist_verAngle = 90;
unsigned int _wrist_rotAngle = 90;
unsigned int _gripperAngle =73; //closed

/*
   Step Delay: a milliseconds delay between the movement of each servo.  Allowed values from 10 to 30 msec.
   M1=base degrees. Allowed values from 0 to 180 degrees
   M2=shoulder degrees. Allowed values from 15 to 165 degrees
   M3=elbow degrees. Allowed values from 0 to 180 degrees
   M4=wrist vertical degrees. Allowed values from 0 to 180 degrees
   M5=wrist rotation degrees. Allowed values from 0 to 180 degrees
   M6=gripper degrees. Allowed values from 10 to 73 degrees. 10: the toungue is open, 73: the gripper is closed.
  */

void BraccioMove( const std_msgs::UInt8MultiArray& angleArray){
  _baseAngle = (unsigned int)angleArray.data[0];
  _shoulderAngle = (unsigned int)angleArray.data[1];
  _elbowAngle = (unsigned int) angleArray.data[2];
  _wrist_verAngle = (unsigned int)angleArray.data[3];
  _wrist_rotAngle = (unsigned int)angleArray.data[4];
  _gripperAngle = (unsigned int)angleArray.data[5];
}

ros::Subscriber<std_msgs::UInt8MultiArray> sub("joint_array", &BraccioMove);

void setup()
{ 
  nh.getHardware()->setBaud(1000000);
  nh.initNode();
  nh.subscribe(sub);
  Braccio.begin();
}

void loop() {  
  Braccio.ServoMovement(10,_baseAngle,_shoulderAngle,_elbowAngle,_wrist_verAngle,_wrist_rotAngle,_gripperAngle);
  nh.spinOnce();
  //delay(1);
}
