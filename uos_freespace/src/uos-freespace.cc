#include <math.h>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#define INVPIDIV180          57.29578               /*   180 / pi   */
#define EPSILON              0.000001               // vergleich von 2 doubles
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

//set laserscan_pos to -1 if the scaner is upside down, if not set to 1
#define LASERSCAN_POS  -1 //laser scanner upside down
//#define LASERSCAN_POS  1

double max_vel_x, max_rotational_vel;
ros::Publisher vel_pub;
//state_turn needs to be global or otherwise the state will allwayes set to 0 when autonomous_behave() gets called 
int state_turn = 0;
//turn_omega is used to determine the maximum angular velocity? Should it be global?
double turn_omega = 0.0;


//find the maximum distance infront of the robot. The orientation weight is used to determine if its infront and far away
double calc_freespace(const sensor_msgs::LaserScan::ConstPtr &laser)
{
  unsigned int i;
  double phi;
  double sinsum = 0.0, cossum = 0.0;
  double orientation_weight;
  double alpha = 0.0;

  //testen

  // printf("*****************\n");
  for (i = 0; i < laser->ranges.size(); i++) {
    // -M_PI/2.0 fuer die verschiebung auf -90 bis 90
    // macht in der summe +M_PI/2.0#

    // phi = (double)i / (laser->ranges.size() - 1) * M_PI - M_PI/2.0;
    phi = laser->angle_min + i * laser->angle_increment;

    //the arg of cos should not be smaler then phi/2 otherwise the robot will get instable when there are obstacles close to the side of the robot
    //rang-arg determines where the max change of the exp function is
    // /0.5 change the width of the exp function. Smaller value means bigger change.
    orientation_weight = cos(phi/2) * 1.0/(1.0+exp(-(laser->ranges[i]-1)/0.5));
    sinsum += sin(phi) * orientation_weight;
    cossum += cos(phi) * orientation_weight;
  }
  // printf("*****************\n");
  // dont know whats happening here, why do we check if cossum is bigger then Epsilon?
  if (fabs(cossum) > EPSILON) alpha = atan2(sinsum, cossum);

  return LASERSCAN_POS*alpha;
}

int SICK_Check_range(const sensor_msgs::LaserScan::ConstPtr &laser,
    double xregion, double yregion,
    int *index_to_obstacle, double *distance_to_obstacle)
{
  unsigned int i;
  double general_distance = 65536.0;
  *distance_to_obstacle = 65536.0;
  *index_to_obstacle = -1;
  double removeAngle = 45;
  double iAngle = (((removeAngle)*(3.14/180))/laser->angle_increment);
  int limit = int(floor(iAngle));

  //The laser scanner detects part of the robot frame.
  //ignore data from min angle to min angle plus limit and from max_angle minus limit
  for (i = fabs(limit); i < (laser->ranges.size() - fabs(limit)); i++) {
		double x = laser->ranges[i] * cos(laser->angle_min + i * laser->angle_increment);
		double y = laser->ranges[i] * sin(laser->angle_min + i * laser->angle_increment);
		if (laser->ranges[i] > 0.1) { // if the range is smaller then a specific amount ignore it. Does this still make sense?
		  if ((x < xregion) && (fabs(y) < (yregion / 2))) { // Is there something in a specific area infront of the robot?
			if (*distance_to_obstacle > x) {
			  *distance_to_obstacle = x;	//Found obstacle, set distance to it
			  *index_to_obstacle = i;
			}
		  }
		  if ((fabs(y) < (yregion / 2))) { //Is there something in general infront of the robot?
			if (general_distance > x) {
			  general_distance = x;		   //Set general distance to the closest object infront of the robot
			}
		  }
		}
	}

  if (*index_to_obstacle == -1) *distance_to_obstacle = general_distance;
  return 0;
}
//Statemachine
//Drive into the general direction of the freespace infront of the Robot
//If there is an obstacle infront of the robot turn away from it
//X axis is pointing out of the front of the robot in direction
//Y axis is pointing out of the axis of the right front wheel
//Z axis is pointing to the roof
void autonomous_behave(const sensor_msgs::LaserScan::ConstPtr &laser)
{
  //Here the region for the obstacle avoidance is defined
  //       ^
  //       |
  //    [XRegion]   
  //       |
  // | <-YRegion-> |
  //       | 
  //    ___+___      +laserscanner
  //   | Robot |
  double XRegion = 0.5; //[m] defines the region in x direction infront of the robot
  double YRegion = 0.5; //[m] defines the region in y direction infront of the robot
  double DTOStopTurning = 1; //[m] defines when to start to turn into freespace
  double DTOStartTurning = 0.5; //[m] defines when to start to turn away form an obstacle
  int index_to_obstacle = -1;
  double Front_Max_Distance = 0;
  
  double u_turning = 0.2 * max_vel_x;    //decrease the speed while turning
  double distance_to_obstacle = INFINITY;
  double omega = calc_freespace(laser); //find the freespace infront of the robot and return the angle to the direction of the freespace
  double u = max_vel_x;                 //define maximum velocity of the robot
  
  for(unsigned int i = 0; i < laser->ranges.size(); i++) {
    if(laser->ranges[i] > Front_Max_Distance) {
      Front_Max_Distance = laser->ranges[i];
    }
  }
  //look for obstacles infront of the robot
  SICK_Check_range(laser,
      XRegion,
      YRegion,
      &index_to_obstacle,
      &distance_to_obstacle);

  // slow down if obstacle
  if (index_to_obstacle != -1) {
    u = distance_to_obstacle / (XRegion * max_vel_x);
  }

  // handle turning mode
  //turn if obstacle is infront
  if (state_turn == 1) {
    if ( (distance_to_obstacle > DTOStopTurning)) {
        state_turn = 0;                 // end turning
        
      } else {
            if (omega > 0.0) {
				if (omega < max_rotational_vel)
					omega = max_rotational_vel - omega;
				else 
					omega = M_PI - omega;
			}
			else if (omega < 0.0) {
				if (omega > max_rotational_vel)
					omega = -max_rotational_vel - omega;
				else 
					omega = -M_PI - omega;
			}
        //omega = turn_omega;             // turn
        u = min(u, u_turning);          //set velocity to u if its not higher then the maximum velocity
      }
  }
  // detect end of corridor use StopTurn val
  if (((Front_Max_Distance < DTOStopTurning)
        || (distance_to_obstacle < DTOStartTurning))
      && (state_turn == 0) ) {
 
    // start turning
    // turn away from obstacle
    if (omega > 0.0) {
      omega = min(omega,max_rotational_vel);
    }
    else if (omega < 0.0) {
      omega = max(omega,-max_rotational_vel);
    }
    state_turn = 1;
    //turn_omega = omega;
    u = min(u, u_turning);
  }


  geometry_msgs::Twist vel;
  vel.linear.x = u;
  vel.angular.z = omega;
  vel_pub.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uos_freespace");

  ros::NodeHandle nh;
  ros::NodeHandle nh_ns("~");

  nh_ns.param("max_vel_x", max_vel_x, 0.3);
  nh_ns.param("max_rotational_vel", max_rotational_vel, 0.3);

  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber laser_sub = nh.subscribe("scan", 10, autonomous_behave);

  ros::spin();
}
