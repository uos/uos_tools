# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 3.8)
project(uos_maps)
# Load catkin and all dependencies required for this package
find_package(ament_cmake REQUIRED)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()