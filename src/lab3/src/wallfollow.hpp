#pragma once
#include <ros/ros.h>
#include <std_msgs/UInt8.h> // include for switch
#include <cardriver/infrared.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>  //command publish message type
#include <iostream>
#include <math.h>

#define TEST_SW //uncomment this line to test switches callback
#define TEST_IR //uncomment this line to test infrared callback
//#define TEST_IMU//uncomment this line to test imu callback

namespace lab3 { //namespance lab3

class WallFollow
{
private:
  //callback functions:
  void switchCallback(const std_msgs::UInt8::ConstPtr& msg);
  void infraredCallback(const cardriver::infrared::ConstPtr& msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void timerCallback(const ros::SteadyTimerEvent& e);
  //private variables:
  bool switches[6] = {0,0,0,0,0,0};
  uint8_t infrared[3] = {0,0,0};
  int16_t angularVel[3] = {0,0,0};
  int16_t linearAccel[3] = {0,0,0};
  int wallFollowInput;
public:
  //functions
  WallFollow(ros::NodeHandle& nh,int input);
  //public variables
  ros::Publisher cmdPub;  // command publisher
  ros::Subscriber swSub, irSub, imuSub; // subscribers
  ros::SteadyTimer timer;

};

} // end of namespace lab3


// TEST_IR sensor:
// RANGE:   min: 4 inches;   max: 24 inches

// TEST_SW sensor:
// FORMAT: uint8 data  i.e.: xxxxxxxx, unsigned 8-bit int
// Cases: (by echo /ti/switches, from left to right when car is facing you)
// No trigger: 00111111
// 1st trigger: 00111110
// 2nd trigger: 00111101
// 3nd trigger: 00111011
// 4nd trigger: 00110111
// 5nd trigger: 00101111
// 6nd trigger: 00011111



