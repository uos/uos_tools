# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(uos_freespace)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs
)


add_executable(uos_freespace src/uos-freespace.cc)

target_link_libraries(uos_freespace ${catkin_LIBRARIES})

install(TARGETS
    uos_freespace
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
