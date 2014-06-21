# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(joint_state_muxer)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS urdfdom_py)

include_directories( ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS urdfdom_py
)
