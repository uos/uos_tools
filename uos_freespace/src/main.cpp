#include <ros/ros.h>
#include <uos_freespace/uos_freespace.h>

int main(int argc, char** argv) {
	
  ros::init(argc, argv, "uos_freespace");
  FreeSpace fs;
  
  ros::spin();
  return(0);
}
