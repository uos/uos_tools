# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(simple_approximate_time_synchronizer)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy message_filters)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS rospy message_filters
)
