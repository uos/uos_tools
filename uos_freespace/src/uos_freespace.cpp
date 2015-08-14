/*
 * uos_freespace
 * Author: Marat "Peter" Purnyn
 * Created 11/8/2015 (based on earlier work uos-freespace.cc)
 */
#include <uos_freespace/uos_freespace.h>
#include <cmath>
#include <cstdio>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/QuaternionStamped.h>

#define EPSILON 0.000001 // arbitrarily small value for cos limit to prevent numerical instability
#define min(a,b) ((a)<(b)?(a):(b)) //carried over from old code, possible cmath is enough
#define max(a,b) ((a)>(b)?(a):(b))
#define LASERSCAN_OPENING_ANGLE_WIDTH 270 //the viewing angle of the laserscanner (possibly only for lms100)
#define LASERSCAN_DEADZONE 45 //the angle range one either side of the scanner to ignore

FreeSpace::FreeSpace():
	turn_state_(0),
	scanner_orientation_(0),
	turn_omega_(0),
	private_nh_("~")
{
	//sets the limits for the speed of the navigation
	private_nh_.param("max_vel_x_", max_vel_x_, 0.5);
	private_nh_.param("max_rotational_vel_", max_rotational_vel_, 0.3);
	
	turn_state_ = 0; 
					
	scanner_orientation_ = 0; 
	
	//publishers and subscribers
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	laser_sub_ = nh_.subscribe("scan", 10, &FreeSpace::autonomousBehaviour, this);
	ROS_DEBUG("constructor");
}

double FreeSpace::calcFreespace(const sensor_msgs::LaserScan::ConstPtr &laserscan){
	unsigned int i; //index of a laserscan point
	double phi;		//angle of i
	double sinSum = 0.0, cosSum = 0.0;
	double orientationWeight;
	double alpha = 0.0;
	
	for (i = 0; i < laserscan->ranges.size(); i++) {
		if(laserscan->ranges[i] > 0.1){
			phi = laserscan->angle_min + i * laserscan->angle_increment;
			
			//the arg of cos should not be smaller than phi/2 otherwise 
			//the robot will be unstable when obstacles are close to its side. 
			//The exponential function is used to weight the range information.
			//The farther an object is the larger its weight.
			
			orientationWeight = cos(phi/2) * 1.0/(1.0+exp(-(laserscan->ranges[i]-1)/0.5));
			sinSum += sin(phi) * orientationWeight;
			cosSum += cos(phi) * orientationWeight;
		}
	}
	if (fabs(cosSum) > EPSILON){
		alpha = atan2(sinSum, cosSum);
	}
	return alpha;
}

int FreeSpace::isInvertedScannerCheck(const sensor_msgs::LaserScan::ConstPtr &laserscan){
	// To account for lasers that are mounted upside-down, we determine the
	// min, max, and increment angles of the laser in the base frame.
	tf::Quaternion q;
	q.setRPY(0.0, 0.0, laserscan->angle_min);
	tf::Stamped<tf::Quaternion> min_q(q, laserscan->header.stamp,laserscan->header.frame_id);
	q.setRPY(0.0, 0.0, laserscan->angle_max);
	tf::Stamped<tf::Quaternion> max_q(q, laserscan->header.stamp,laserscan->header.frame_id);
	
	try	{
		//wait for the tf frames to be available then transform the laserscan frame to the base frame
		//this makes it so all scanners data is read as right-side up
		tf_.waitForTransform("base_link", "laser",laserscan->header.stamp,ros::Duration(1.0));
		tf_.transformQuaternion("base_link", min_q, min_q);
		tf_.transformQuaternion("base_link", max_q, max_q);
	}
	catch(tf::TransformException& e) {
		//if there is an error transforming, keep the orientation unset to try again
		ROS_WARN("Unable to transform min/max laser angles into base frame: %s",e.what());
		return 0;
	}
	
	ROS_DEBUG("Successfully transformed min/max laser angles into base frame");
	
	//calculate the angle increment of the scan with respect the base frame
	double minAngle = tf::getYaw(min_q);
	double maxAngle = tf::getYaw(max_q);
	double baseFrameAngleIncrement = (maxAngle - minAngle) / laserscan->ranges.size();
	
	ROS_DEBUG("Laser angles in base frame: min: %.3f max: %.3f inc: %.3f", minAngle, maxAngle, baseFrameAngleIncrement);
	if (baseFrameAngleIncrement < 0){
		//upside-down
		ROS_DEBUG("Inverting scan");
		return -1;
	}
	return 1; //if not return by now, then assume right side up
}

int FreeSpace::checkRange(
	const sensor_msgs::LaserScan::ConstPtr &laserscan, 
	double xregion, double yregion,
    int *indexOfObstacle, 
    double *distanceToObstacle)
{		
	unsigned int i; //index of laser point
	double generalDistance = 65536.0; //starting max distance
	*distanceToObstacle = 65536.0; //starting distance to obstacle
	*indexOfObstacle = -1; //starting index of obstacle
	
	//calculate the number of laserpoints in the deadzone
	int deadzonePoints = int(floor((((LASERSCAN_DEADZONE)*(M_PI/180))/laserscan->angle_increment)));
	ROS_WARN("deadzonepoints :%d", deadzonePoints);
	//The laser scanner detects part of the robot frame.
	//ignore laserpoints in deadzone on the left and right of the scan
	for (i = fabs(deadzonePoints); i < (laserscan->ranges.size() - fabs(deadzonePoints)); i++) {
		if(laserscan->ranges[i] > 0.1){
			double x = laserscan->ranges[i] * cos(laserscan->angle_min + i * laserscan->angle_increment);
			double y = laserscan->ranges[i] * sin(laserscan->angle_min + i * laserscan->angle_increment);
			if ((x < xregion) && (fabs(y) < (yregion / 2))) { 
				// Is there something in the region in front of the robot?
				if (*distanceToObstacle > x) {
				//Found obstacle, record distance and index
				  *distanceToObstacle = x;	
				  *indexOfObstacle = i;
				  ROS_WARN("set index to obstacle %d", i);
				}
			} //is there something in the generel direction?
			if ((x > xregion) && (fabs(y) < (yregion / 2))) { 
				//Is there something ahead of the robot?
				if (generalDistance > x) {
				//Set general distance to the closest object infront of the robot
				  generalDistance = x;	
				  //ROS_WARN("set general distance");	   
				}
			}
		}
	}
	
	if (*indexOfObstacle == -1){ 
		//There is no obstacle, so reset the distance.
		*distanceToObstacle = generalDistance;
	}
	ROS_WARN("distance to obstacle :%f", *distanceToObstacle);
	ROS_WARN(" index obstacle :%d", *indexOfObstacle);
	return 0;		
}

// Statemachine
// Drive into the general direction of the freespace infront of the Robot
// If there is an obstacle infront of the robot turn away from it
// X axis is pointing forward
// Y axis is pointing left
// Z axis is pointing up
void FreeSpace::autonomousBehaviour(const sensor_msgs::LaserScan::ConstPtr &laserscan){
	ROS_DEBUG("autonomousBehaviour");
	// region is a quadilatiral in front of the robot
	// define the size
	double xregion = 0.5; // x is in forward
	double yregion = 0.5;  // y is left
	double DTOStopTurning = 1.0; // defines when to start to turn into freespace
	double DTOStartTurning = 0.5; // defines when to start to turn away form an obstacle
	int indexOfObstacle = -1;
	double frontMaxDistance = 0;

	double uTurning = 0.2 * max_vel_x_;    //decrease the speed while turning
	double distanceToObstacle = INFINITY;
	double omega = calcFreespace(laserscan); // find the freespace infront of the robot and return the angle to the direction of the freespace
	double u = max_vel_x_;                 // define maximum velocity of the robot
	
	//ROS_WARN("behave maxpoints :%d", laserscan->ranges.size());
  
	for(unsigned int i = 0; i < laserscan->ranges.size(); i++) {
		if(laserscan->ranges[i] > frontMaxDistance) {
			frontMaxDistance = laserscan->ranges[i];
		}
	}
	
	if(scanner_orientation_ == 0) {
		scanner_orientation_ = isInvertedScannerCheck(laserscan);
		ROS_DEBUG("scanner_orientation_: %d", scanner_orientation_);
	}

	// look for obstacles infront of the robot
	checkRange(laserscan,xregion,yregion,&indexOfObstacle,&distanceToObstacle);

	// slow down if obstacle
	if (indexOfObstacle != -1) {
		u = max(distanceToObstacle - 0.2 / (xregion * max_vel_x_),0);
	}

	// handle turning mode
	// turn if obstacle is infront
	if (turn_state_ == 1) {
		ROS_WARN("turn_state: 1");
		if ( (distanceToObstacle > DTOStopTurning)) {
			turn_state_ = 0;                 // end turning
		} 
		else {
			//turn slower if obstacle is infront
				ROS_WARN("turning away");
		}  
	  u = min(u, uTurning);          //set velocity to u if its not higher then the maximum velocity
	}
	// detect end of corridor use StopTurn val
	if (((frontMaxDistance < DTOStopTurning) || (distanceToObstacle < DTOStartTurning)) && (turn_state_ == 0) ) {
		// start turning
		// turn away from obstacle
		if (indexOfObstacle > laserscan->ranges.size() / 2) {
			turn_omega_ = -max_rotational_vel_;
			//omega = min(omega,max_rotational_vel_); //omega should not exceed max_rotational_vel_
			ROS_WARN("turning_left");
		}
		else if (indexOfObstacle <= laserscan->ranges.size() / 2 && indexOfObstacle >= 0) {
			turn_omega_ = max_rotational_vel_;
			
			//omega = max(omega,-max_rotational_vel_);
			ROS_WARN("turning_right");
		}
		turn_state_ = 1;
		u = min(u, uTurning);
	}
	if (turn_state_ == 1){
		omega = turn_omega_;
	}
	
	geometry_msgs::Twist vel;
	vel.linear.x = u;
	vel.angular.z = scanner_orientation_ * omega;
	vel_pub_.publish(vel);
}
