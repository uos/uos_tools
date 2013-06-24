/*This program will run the robot in three different speeds
 The buttons array shows which buttons are currently depressed on the Wiimote device.
 The position mapping is as follows:
 Position   Button Name
 0         1
 1         2
 2         A
 3         B (toggle button on back of device)
 4         Plus
 5         Minus
 6         Rocker Left
 7         Rocker Right
 8         Rocker Up
 9         Rocker Down
 10        HOME
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

double max_vel_x, max_rotational_vel;
ros::Publisher vel_pub;

void ps3joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;

  int c; // assigning three cases based on three buttons on the wii remote
  if (joy->buttons[0])
    c = 1;
  else if (joy->buttons[1])
    c = 2;
  else if (joy->buttons[3])
    c = 3;

  //choosing speed and assigning it to max_vel
  switch (c)
  {
    case 1:
      max_vel_x = 0.1;
      break;
    case 2:
      max_vel_x = 0.5;
      break;
    case 3:
      max_vel_x = 1.5;
      break;
  }

  // assigning x and z value in the Twist message
  if (joy->buttons[8])
  {
    vel.linear.x = max_vel_x;
  }
  else if (joy->buttons[9])
  {
    vel.linear.x = -max_vel_x;
  }
  else if (joy->buttons[6])
  {
    vel.angular.z = max_rotational_vel;
  }
  else if (joy->buttons[7])
  {
    vel.angular.z = -max_rotational_vel;
  }
  else

  {
    vel.linear.x = 0;
    vel.angular.z = 0;
  }

  vel_pub.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uos_diffdrive_teleop_wiimote");

  ros::NodeHandle nh;
  ros::NodeHandle nh_ns("~");

  nh_ns.param("max_vel_x", max_vel_x, 1.5);
  nh_ns.param("max_rotational_vel", max_rotational_vel, 1.5);

  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber ps3joy_sub = nh.subscribe("joy", 10, ps3joyCallback);

  ros::spin();
}

