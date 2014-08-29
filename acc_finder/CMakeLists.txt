# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(acc_finder)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs nav_msgs)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS rospy geometry_msgs nav_msgs
)

install(PROGRAMS
    nodes/acc_finder.py
    nodes/min_max_finder.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
