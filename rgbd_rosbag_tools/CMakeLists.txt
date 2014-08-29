# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(rgbd_rosbag_tools)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy rosbag sensor_msgs cv_bridge)

catkin_package(
    CATKIN_DEPENDS rospy rosbag sensor_msgs cv_bridge
)

install(PROGRAMS
    scripts/add_pointclouds_to_bagfile.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
