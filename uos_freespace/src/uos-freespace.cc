#include <math.h>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#define INVPIDIV180          57.29578               /*   180 / pi   */
#define EPSILON              0.000001               // vergleich von 2 doubles
#define min(a,b) ((a)<(b)?(a):(b))

double max_vel_x, max_rotational_vel;
ros::Publisher vel_pub;

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

    //printf("%d %f \n", i, phi);
    // cos(phi/2) gewichtet die werte nach vorne maximal und am
    // rand entsprechend weniger (hier 0)
    // exp term fuer grosse distance ungefaehr 0 d.h. keine einfluss

    orientation_weight = cos(phi/1.2) * 1.0/(1.0+exp(-(laser->ranges[i]-3.0)/0.3));
    // kann aber nicht oben berechnet werden wegen scalierung von obigen wert
    // printf("%d %lf %f %f\n",i,laser->ranges[i], INVPIDIV180 * phi, orientation_weight);
    sinsum += sin(phi) * orientation_weight;
    cossum += cos(phi) * orientation_weight;
  }
  // printf("*****************\n");

  if (fabs(sinsum) > EPSILON) alpha = atan2(sinsum, cossum);

  // printf("a %f \n",INVPIDIV180 * alpha);
  // hier noch checken ob was null wird
  // if (fabs(Back_sinsum) > EPSILON) alpha += atan2(Back_sinsum,Back_cossum);
  /*
  printf("alpha %f sinsum %f cossum %f \n",
      INVPIDIV180 * alpha,
      sinsum,
      cossum);
  */
  return alpha;
}

int SICK_Check_range(const sensor_msgs::LaserScan::ConstPtr &laser,
    double xregion, double yregion,
    int *index_to_obstacle, double *distance_to_obstacle)
{
  unsigned int i;
  double general_distance = 65536.0;
  *distance_to_obstacle = 65536.0;
  *index_to_obstacle = -1;
  //for (i = 0; i < laser->ranges.size(); i++) {
  for (i = 10; i < (laser->ranges.size() - 10); i++) {
    // new wegen der aufhaengung muss ein minimal wert ueberschritten sein
    // cout<<"sickscanner check range laser->ranges.size(): "<<laser->ranges.size()<<" distance "<<*distance<<" x["<<i<<"] "<<x[i]<<" y["<<i<<"] "<<y[i] <<" xregion "<<xregion<<" yregion "<<yregion<<endl;
    double x = laser->ranges[i] * sin(laser->angle_min + i * laser->angle_increment);
    double y = laser->ranges[i] * cos(laser->angle_min + i * laser->angle_increment);
    if (laser->ranges[i] > 0.1) { // damit wir auch nicht nur uns sehen
      if ((fabs(x) < xregion) && (y < yregion)) { // hier muessen wir uns etwas merken
        if (*distance_to_obstacle > y) {
          *distance_to_obstacle = y;
          *index_to_obstacle = i;
        }
      }
      if ((fabs(x) < xregion)) { // hier muessen wir uns etwas merken
        if (general_distance > y) {
          general_distance = y;
        }
      }
    }
  }
  if (*index_to_obstacle == -1) *distance_to_obstacle = general_distance;
  return 0;
}

void autonomous_behave(const sensor_msgs::LaserScan::ConstPtr &laser)
{

  int state_turn = 0;
  double XRegion = 0.25; //25.0
  double ZRegion = 0.56; // 56.0
  double DTOStopTurning = 0.60; //60.0
  double DTOStartTurning = 0.50; //50.0
  int index_to_obstacle = -1;
  double Front_Max_Distance = -1; //maximaler laserstrahl
  double turn_omega = 0.0;
  double u_turning = 0.2 * max_vel_x;
  double distance_to_obstacle = INFINITY;
  double omega = calc_freespace(laser);
  double u = max_vel_x;
  for(unsigned int i = 0; i < laser->ranges.size(); i++) {
    if(laser->ranges[i] > Front_Max_Distance) {
      Front_Max_Distance = laser->ranges[i];
    }
  }
  //cout<<" autonomous behave omega: "<<omega<<" max_vel_x "<<max_vel_x<<endl;
  // check if obstacle in front (xregion: width, yregion: length)
  // virtual roadway
  SICK_Check_range(laser,
      XRegion,
      ZRegion,
      &index_to_obstacle,
      &distance_to_obstacle);

  // slow down if obstacle
  if (index_to_obstacle != -1) {
    //cout<<" autonomous obstacle"<<endl;
    u = distance_to_obstacle / ZRegion * max_vel_x;
  }

  // handle turning mode
  if (state_turn == 1) {
    //cout<<" autonomous state turn"<<endl;
    if ( (distance_to_obstacle > DTOStopTurning)
        // && (fabs(omega) < M_PI*10.0/180.0) )
      ) {
        state_turn = 0;                 // end turning
        //status->AntiWindup = 0.0;               // reset pid integration
      } else {
        omega = turn_omega;             // turn
        u = min(u, u_turning);
      }
  }
  // detect end of corridor use StopTurn val
  if (((Front_Max_Distance < DTOStopTurning)
        || (distance_to_obstacle < DTOStartTurning))
      && (state_turn == 0) ) {
    //cout<<" autonomous end of corridor FMD "<<Front_Max_Distance<<endl;
    // start turning
    // turn away from obstacle
    if (omega > 0.0) {
      omega = 0.999 * M_PI;
    }
    if (omega < 0.0) {
      omega = -0.999 * M_PI;
    }
    state_turn = 1;
    turn_omega = omega;
    u = min(u, u_turning);
    //status->AntiWindup = 0.0;                 // reset pid integration
  }

  //cout << u << " " << omega << " " << state_turn << " " << distance_to_obstacle << endl;

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

  nh_ns.param("max_vel_x", max_vel_x, 0.9);
  nh_ns.param("max_rotational_vel", max_rotational_vel, 1.0);

  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber laser_sub = nh.subscribe("scan", 10, autonomous_behave);

  ros::spin();
}
