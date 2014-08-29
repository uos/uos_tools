# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tf_publisher_gui)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy tf)

catkin_package(
    CATKIN_DEPENDS rospy tf
)

install(PROGRAMS
    tf_publisher_gui
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
    launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
