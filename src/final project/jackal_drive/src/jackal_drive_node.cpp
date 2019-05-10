//SE498 final project

#include <iostream>
#include <stdlib.h> 
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdio.h>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/JointState.h"
#include <math.h>
#define PI 3.14159265


ros::Publisher jackal_drive_publisher;
ros::Subscriber jackal_laser_scan_sub;
ros::Subscriber jackel_imu_sub;

void LocalizeCallback(const sensor_msgs::LaserScan & msg);
void ImuCallback(const sensor_msgs::JointState & msg);
float dist_init;
bool avoidance_mode = false; 
using namespace std;



int main(int argc, char **argv){

	ros::init(argc, argv, "jackal_drive");
	ros::NodeHandle nh;

	jackal_drive_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	jackal_laser_scan_sub = nh.subscribe("/front_scan", 1, LocalizeCallback);
	jackel_imu_sub = nh.subscribe("/joint_states", 1, ImuCallback);
	ros::spin();

	return 0;
}
	
void ImuCallback(const sensor_msgs::JointState & msg) {
	dist_init = msg.position[0];
	//cout << "position.x: " << msg.position[0]<< "position.y: " << msg.position[1]<<endl; 
}


void LocalizeCallback(const sensor_msgs::LaserScan & msg) {
	// Basic LiDAR info variables
	size_t size = msg.ranges.size();
	vector<float> range_data = msg.ranges;
	float angle = 0.0f;
	float dest_ori = 0.0f, dest_dist = 0.0f, total_dist_l = 0.0f, total_dist_r = 0.0f;
	float cur_ori = 0.0f, cur_dist = 0.0f;
	int count_l = 0, count_r = 0, count_front = 0;

	// Wall Following related variables
	float desired_dist = 1.25f;
	float dist_min = 100.0f;
	float pose_angle = 0.0f; // Current Jakcal Heading angle

	// Obstacle Avoidance related variables
	const float avoid_range = 2.0f;
	const int avoid_counter_thre = 5;
	const float good_angle_thre = 0.07f; // 5 degree around
	int avoid_counter = 0;
	float prev_good_angle = 0.0f;
	vector<float> curr_good_region;
	vector<vector<float> > good_regions;
	

	// For-loop: 
	// 1. under Jackal's frame, detect the left & right wall distance to Jackal  
	// 2. detect and save Jackal's current pose
	// 3. detect good_regions for obstacle avoidance
	for (int i = 0; i < size; ++i) {
	    if(range_data[i] < msg.range_max && range_data[i] > msg.range_min + 0.000001f) {
		// current LiDAR point's angle and dist
	      cur_ori = angle;
				cur_dist = range_data[i];

				// under current Jackal's frame: detect good regions for obstacle avoidance
				if (cur_dist > avoid_range) { // it is a hole
					if (curr_good_region.size() == 0 && abs(prev_good_angle - cur_ori) > good_angle_thre) {
						curr_good_region.push_back(cur_ori);	
					}
					prev_good_angle = cur_ori;
				}
				else if (curr_good_region.size() == 1 && abs(prev_good_angle - cur_ori) > good_angle_thre) { // it is a block
					curr_good_region.push_back(cur_ori);
					good_regions.push_back(curr_good_region);
					curr_good_region.clear();
				}
				// push back the last region into the good_regions
				if (cur_dist > avoid_range && curr_good_region.size() == 1 && abs(prev_good_angle - cur_ori) > good_angle_thre) {
					curr_good_region.push_back(cur_ori);
					good_regions.push_back(curr_good_region);
					curr_good_region.clear();			
				}

				// under current Jackal's frame: detect left and right dist
				if ((cur_ori < 4.97418828333)&&(cur_ori > 4.45058951667)){ // 3/2 * PI around
					count_l ++;
					total_dist_l += cur_dist*sin(cur_ori-PI);
				}
				if ((cur_ori < 4.97418828333-PI)&&(cur_ori > 4.45058951667-PI)){
					count_r++;
					total_dist_r += cur_dist*sin(cur_ori);
					// under world frame: detect current Jackal Heading pose 
					// based on right dist data
					if (dist_min > cur_dist) {
						dist_min = cur_dist;
						pose_angle = -cur_ori + PI/2;
					}
				}
  		}
	    angle += msg.angle_increment;
	}

	if (count_l > 0) {total_dist_l /= count_l;}
	if (count_r > 0) {total_dist_r /= count_r;}

	cout << "good_regions.size(): " <<  good_regions.size() << endl;
	for (int i = 0; i < good_regions.size(); ++i) {
		float center_angle = (good_regions[i][0] + good_regions[i][1]) / 2;
		float angle_width = abs(good_regions[i][0] - good_regions[i][1]);
		cout<<good_regions[i][0]<<";"<<good_regions[i][1]<<endl;
		good_regions[i][0] = center_angle;
		good_regions[i][1] = angle_width;
		cout << "center angle:" << center_angle << ", angle width = " << angle_width << endl; 
	}

	/////////////////////////////////////////////////////////////////////////////
	////// Control Strategies for Wall Following and Obstacle Avoidance /////////
	/////////////////////////////////////////////////////////////////////////////

	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x = 0.6;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;	
	if (dist_init < 20) {vel_msg.angular.z = fmax(fmin(-1 * (total_dist_r-desired_dist) * 0.3, 1), -1);}	
	else if (good_regions.size() == 1) {
		
		float angle_err = good_regions[0][0] - PI;
		cout<<"Angle"<<angle_err<<endl;
		vel_msg.angular.z = fmax(fmin(angle_err * 0.35, 2), -2); //constraint 
	}
	// Wall Following Controller: Fast Mode
	else if (good_regions.size() == 2) {
			vel_msg.linear.x = 0.3;
			float center1 = good_regions[0][0];
			float center2 = good_regions[1][0];
			float width1 = good_regions[0][1];
			float width2 = good_regions[1][1];
			float angle_err = 0;
			if (width2 > width1) {angle_err = center2-PI;}
			else {angle_err = center1-PI;}
			cout<<"Angle"<<angle_err<<endl;
			vel_msg.angular.z = fmax(fmin(angle_err * 0.35, 2), -2); //constraint 
		}
	// // Wall Following Controller: Slow Mode
//	else if (good_regions.size() == 3) {
//			vel_msg.linear.x = 0.3;
//			vector<vector<float>> forward_regions;
//			for (int i = 0; i < good_regions.size(); ++i) {
//				if (good_regions[i][0] < 1.5*PI && good_regions[i][0] > 0.5*PI) {
//						forward_regions.push_back(good_regions[i]);
//				}
//			}
//			sort(forward_regions.begin(),forward_regions.end(),[](const vector<float>& a, const vector<float>& b) {return a[1]<b[1];});
//			float steer_input = (forward_regions[0][0]-PI) * 0.35;
//			cout<< forward_regions[0][1]<<";"<<forward_regions[1][1]<<endl;
//			vel_msg.angular.z = fmax(fmin(steer_input, 2), -2); //constraint 	
//	}
	else {
		vel_msg.angular.z = fmax(fmin((total_dist_l-desired_dist)*0.35+fmax(fmin(pose_angle*(-1.2),0.45),-0.45),2),-2);
	}
	// cout << "linear.x: " << vel_msg.linear.x << ", vel_msg.angular.z: " << vel_msg.angular.z << endl;	
	// cout<< dist_init<<";"<<total_dist_l <<";"<< total_dist_r<<";"<<vel_msg.angular.z<<endl;
	jackal_drive_publisher.publish(vel_msg);
}
