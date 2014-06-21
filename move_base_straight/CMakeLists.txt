# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(move_base_straight)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS actionlib rospy tf std_msgs actionlib_msgs move_base_msgs)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS actionlib rospy tf std_msgs actionlib_msgs move_base_msgs
)
