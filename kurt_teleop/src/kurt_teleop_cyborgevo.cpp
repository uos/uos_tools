#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>

int linear, angular;
double l_scale, a_scale;
ros::Publisher vel_pub;

void cyborgevoCallback(const joy::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  vel.angular.x = a_scale * joy->axes[angular];
  vel.linear.x = l_scale * joy->axes[linear];
  vel_pub.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kurt_teleop_cyborgevo");

  ros::NodeHandle nh;
  ros::NodeHandle nh_ns("~");

  nh_ns.param("axis_linear", linear, 1);
  nh_ns.param("axis_angular", angular, 0);
  nh_ns.param("scale_angular", a_scale, 2.0);
  nh_ns.param("scale_linear", l_scale, 2.0);

  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber cyborgevo_sub = nh.subscribe("joy", 10, cyborgevoCallback);

  ros::spin();
}

