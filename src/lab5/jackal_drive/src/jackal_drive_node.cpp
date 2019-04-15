#include <iostream>
#include <stdlib.h> 
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdio.h>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

ros::Publisher jackal_drive_publisher;
ros::Subscriber jackal_laser_scan_sub;

void LocalizeCallback(const sensor_msgs::LaserScan & msg);
using namespace std;


int main(int argc, char **argv){

	ros::init(argc, argv, "jackal_drive");
	ros::NodeHandle nh;

	jackal_drive_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	jackal_laser_scan_sub = nh.subscribe("/front_scan", 1, LocalizeCallback);

	ros::spin();

	return 0;
}
	

void LocalizeCallback(const sensor_msgs::LaserScan & msg) {
	float angle = 0.0f;
	float dest_ori = 0.0f, dest_dist = 0.0f;
	int count = 0;

	size_t size = msg.ranges.size();
	vector<float> range_data = msg.ranges;
	

	for (int i = 0; i < size; ++i) {
	    if(range_data[i] < msg.range_max && range_data[i] > msg.range_min + 0.000001f) {
		count++;
	        dest_ori += angle;
		dest_dist += range_data[i];
    	    }
	    angle += msg.angle_increment;
	}


	if (count > 0) {
	    dest_ori /= count;
	    dest_dist /= count;
	}
	    

	ROS_INFO("\n\n\n*Let Robot keep moving*\n");

	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x = 0.1 * dest_dist;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 1.0f * (dest_ori - 3.1415926f);

	if (vel_msg.linear.x < 0.5) {
		vel_msg.linear.x = 1.0;
		vel_msg.angular.z = 0;
	}

	jackal_drive_publisher.publish(vel_msg);
	cout << "linear.x: " << vel_msg.linear.x << ", vel_msg.angular.z: " << vel_msg.angular.z << endl;
}

