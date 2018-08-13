/*
 * uos_freespace
 * Author: Marat "Peter" Purnyn
 * Created 11/8/2015 (based on earlier work uos-freespace.cc)
 */
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "tf/transform_listener.h"
#include <string>

class FreeSpace {
	public:
			FreeSpace();
	private:
		//variables
			double max_vel_x_, max_rotational_vel_; //max velocity of robot
			double turn_omega_; //this is used to keep track of the turning angle of the robot when turn_state_ = 1;
			int turn_state_; 	//if robot gets too close to an object in front, it switches to turn mode
								//it keeps turning until there is no more obstacle and switches out of turn mode.
			int scanner_orientation_; //0 for initial, 1 for right side up, -1 for upside-down laserscanner
			std::string tf_prefix_; // optional tf_prefix
		//node
			ros::NodeHandle nh_;
			ros::NodeHandle private_nh_;
		//publishers
			ros::Publisher vel_pub_;
		//subscribers
			ros::Subscriber laser_sub_;
		//others
			tf::TransformListener tf_;
			
		//functions
			void autonomousBehaviour(const sensor_msgs::LaserScan::ConstPtr &laserscan);
			double calcFreespace(const sensor_msgs::LaserScan::ConstPtr &laserscan);
			int isInvertedScannerCheck(const sensor_msgs::LaserScan::ConstPtr &laserscan);
			int checkRange(
				const sensor_msgs::LaserScan::ConstPtr &laserscan, 
				double xregion, double yregion,
				int *indexToObstacle, 
				double *distanceToObstacle);	
};
