# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(uos_maps)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED )

catkin_package()

install(DIRECTORY
    maps/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)
