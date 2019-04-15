#include "wallfollow.hpp"

namespace lab3 {

/*!
 * \brief WallFollow::WallFollow
 * WallFollow class constructor, it initializes subscriber and publisher
 * to communicate with the vehicles driver
 * \param nh
 */
WallFollow::WallFollow(ros::NodeHandle &nh, int input):wallFollowInput(input){
  //initialize subscriber for each sensor topic, callback functions are
  //class private functions, 'this' is the pointer to this specific instance
  //a class variable.
  swSub = nh.subscribe("/ti/switches",10,&WallFollow::switchCallback,this);
  irSub = nh.subscribe("/ti/infrared",10,&WallFollow::infraredCallback,this);
  imuSub = nh.subscribe("/ti/imu",10,&WallFollow::imuCallback,this);


  //control message is published as a twist geometry mesasge type
  // only linear.x  and angular.z are used
  cmdPub = nh.advertise<geometry_msgs::Twist>("/ti/cartCtrl",1);

  //initialize timer, such that the callback function will be periodically called
  //! NOTE: the timerCallback is the function you need to modify for this lab.
  int timerRate = 20; // rate in which the timer callback function will be called
  timer = nh.createSteadyTimer(ros::WallDuration(ros::Rate(timerRate)),&WallFollow::timerCallback,this);

}
//************************* 1st assignment ************************
// find and uncomment #define TEST_SW in wallfollow.hpp and compile.
// run the compiled code and test the switches by triggering them individually.
// what is the switch's value when its not being triggered?
// what is the switch's value when its triggered?
/*!
 * \brief WallFollow::switchCallback
 * switch data is embedded in to an 8 bit data, where the 6 least
 * significant bits represents the 6 switches, the rest 2 are un-
 * used. When the switch is triggered the bit corresponding to
 * the switch will be set.
 * \param msg
 */
void WallFollow::switchCallback(const std_msgs::UInt8::ConstPtr &msg){

  switches[0] = msg->data & 0x01;
  switches[1] = msg->data & 0x02;
  switches[2] = msg->data & 0x04;
  switches[3] = msg->data & 0x08;
  switches[4] = msg->data & 0x10;
  switches[5] = msg->data & 0x20;
#ifdef TEST_SW
  for(uint8_t i = 0; i<6; ++i){
    if(!switches[i] )
      std::cout<<"switch \t"<< (int)i + 1<< "\t is triggered" << std::endl;
  }
#endif
}
//************************* 2nd assignment **************************
// find and uncomment #define TEST_IR in wallfollow.hpp and compile.
// run the compiled code and test the infrared sensors by putting
// a piece of obstacle (preferably white) and have a sense of its
// range of capability.
/*!
 * \brief WallFollow::infraredCallback
 * infrared topic callback, it updates infrared sensor data.
 * The data is mapped from 0-255.
 * If you do
 * rostopic echo /ti/infrared
 * you can test the actual range of the infrared,
 * which is 125 for the closest reading and 0 for the furthest reading.
 * \param msg
 */
void WallFollow::infraredCallback(const cardriver::infrared::ConstPtr &msg){

  infrared[0] = msg->left;
  infrared[1] = msg->middle;
  infrared[2] = msg->right;

#ifdef TEST_IR
  std::cout<<"left infrared is \t"<< (int)infrared[0];
  std::cout<<"\t middle infrared is \t" << (int)infrared[1];
  std::cout<<"\t right infrared is \t" << (int)infrared[2] <<std::endl;
#endif

}

//************************** IMU callback *****************************
// imu callback will not be used in this assignment, but feel free to uncomment
// #define TEST_IMU in wallfollow.hpp and test the compiled code.
/*!
 * \brief WallFollow::imuCallback
 * this callback function updates the angular velocity and linear acceleration
 * of the imu sensor.
 * Try to test and find out their units
 * \param msg
 */
void WallFollow::imuCallback(const sensor_msgs::Imu::ConstPtr &msg){

  angularVel[0] = msg->angular_velocity.x;  // msg contain raw 16 bit signed interger data stored in float 64 format
  angularVel[1] = msg->angular_velocity.y;  // msg contain raw 16 bit signed interger data stored in float 64 format
  angularVel[2] = msg->angular_velocity.z;  // msg contain raw 16 bit signed interger data stored in float 64 format
  linearAccel[0] = msg->linear_acceleration.x; // msg contain raw 16 bit signed interger data stored in float 64 format
  linearAccel[1] = msg->linear_acceleration.y; // msg contain raw 16 bit signed interger data stored in float 64 format
  linearAccel[2] = msg->linear_acceleration.z; // msg contain raw 16 bit signed interger data stored in float 64 format

#ifdef TEST_IMU
  for(uint8_t i = 0; i<3; ++i){
    std::cout << "angular velocity and linear accel at \t "<<(int)i<<" \t are \t";
    std::cout << (int)angularVel[i] << "\t and \t" << (int)linearAccel[i] << std::endl;
  }
#endif

}

//*******************3rd assignment (Wall following) ***********************
// Mainly modify this function to control your vehicle
// you can modify other functions and files to suit your need if needed
// You will need to satisfy the following conditions:
// 1. follow left/right wall.
// 2. detect bumper switch hitting low profile obstable
// 3. drive around the low profile obstable
//***************************************************************************
/*!
 * \brief WallFollow::timerCallback
 * You want to modify here to check sensor variables and create your control loop
 * 1. The code should allow your vehicle to follow wall on the left or wall on the right.
 * 2. When a switch is triggered by obstacle, the vehicle should try drive around it.
 * You can ignore "e"
 * \param e
 */
void WallFollow::timerCallback(const ros::SteadyTimerEvent &e){
  //declare message to be published for control
  // all varaibles in cmdMsg are initialized to 0 by default
  geometry_msgs::Twist cmdMsg;
  int errorL = 0;
  int errorM = 0;
  int errorR = 0;
  bool leftFollow = false;
  bool rightFollow = false;
	bool blocked = false;

  errorL = infrared[0] - 70; //error of left sensor to target 100
  errorM = infrared[1] - 100;
  errorR = infrared[2] - 100;
	int time1 = -1;
	int reverseTime = 2;
	

	for (int i=0; i<6; i++) {
		if (!switches[i]) {
				blocked = true;
				time1 = ros::Time::now().toSec();
				break;
		} 
  }

  switch (wallFollowInput) {
  case 0: // left wall follow
    //@HERE replace code below with your own code for left wall follow
    leftFollow = true;
    // direction: angular > 0, left turn
    break;

  case 1: // right wall follow
    //@ HERE replace code below with your own code for right wall follow
    rightFollow = true;
    break;

  default: // should not happen
    break;
  }

	if (leftFollow && !blocked) {	
		if (errorM > -30 && errorM < 0) {
			cmdMsg.angular.z = fmax(fmin(-1 * errorL * 0.15, 2), -2);
			cmdMsg.linear.x = 0.1;
		}
		else if (errorM > 0) {
			time1 = ros::Time::now().toSec();
			while (ros::Time::now().toSec() < reverseTime+time1) {
				cmdMsg.angular.z = -1;
				cmdMsg.linear.x = -0.1;
				cmdPub.publish(cmdMsg);
				ros::Rate rt(20);
				rt.sleep();
			}
			while (ros::Time::now().toSec()-1 < reverseTime+time1) {	
				cmdMsg.angular.z = -1;
				cmdMsg.linear.x = 0.1;
				cmdPub.publish(cmdMsg);
				ros::Rate rt(20);
				rt.sleep();
			}
		}
		else {
			cmdMsg.angular.z = fmax(fmin(-1 * errorL * 0.15, 2), -2);
    	cmdMsg.linear.x = 0.25;
		}
	}
	else if (rightFollow && !blocked) {
		if (errorM > -30 && errorM < 0) {
			cmdMsg.angular.z = fmax(fmin(errorR * 0.15, 2), -2); 
			cmdMsg.linear.x = 0.1;
		}
		else if (errorM > 0) {
			time1 = ros::Time::now().toSec();
			while (ros::Time::now().toSec() < reverseTime+time1) {
				cmdMsg.angular.z = 1;
				cmdMsg.linear.x = -0.1;
				cmdPub.publish(cmdMsg);
				ros::Rate rt(20);
				rt.sleep();
			}
			while (ros::Time::now().toSec()-1 < reverseTime+time1) {	
				cmdMsg.angular.z = 1;
				cmdMsg.linear.x = 0.1;
				cmdPub.publish(cmdMsg);
				ros::Rate rt(20);
				rt.sleep();
			}
		}
		else {
			cmdMsg.angular.z = fmax(fmin(errorR * 0.15, 2), -2); 
    	cmdMsg.linear.x = 0.25;
		}
	}
	else if (leftFollow && blocked){
		while (ros::Time::now().toSec() < reverseTime+time1) {
			cmdMsg.angular.z = -1;
			cmdMsg.linear.x = -0.1;
			cmdPub.publish(cmdMsg);
			ros::Rate rt(20);
			rt.sleep();
		}
		while (ros::Time::now().toSec()-1 < reverseTime+time1) {	
			cmdMsg.angular.z = -1;
			cmdMsg.linear.x = 0.1;
			cmdPub.publish(cmdMsg);
			ros::Rate rt(20);
			rt.sleep();
		}
	}
	else if (rightFollow && blocked){
		while (ros::Time::now().toSec() < reverseTime+time1) {
			cmdMsg.angular.z = 1;
			cmdMsg.linear.x = -0.1;
			cmdPub.publish(cmdMsg);
			ros::Rate rt(20);
			rt.sleep();
		}
		while (ros::Time::now().toSec()-1 < reverseTime+time1) {	
			cmdMsg.angular.z = 1;
			cmdMsg.linear.x = 0.1;
			cmdPub.publish(cmdMsg);
			ros::Rate rt(20);
			rt.sleep();
		}
	}
	

  // publish the control
  cmdPub.publish(cmdMsg);

}

} // end of namespace lab3
