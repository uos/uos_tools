# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(uos_common_urdf)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS urdf xacro gazebo_plugins diffdrive_gazebo_plugin)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS urdf xacro gazebo_plugins diffdrive_gazebo_plugin
)
