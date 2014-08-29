# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(uos_diffdrive_teleop)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
)

add_executable(uos_diffdrive_teleop_cyborgevo src/uos_diffdrive_teleop_cyborgevo.cpp)
add_executable(uos_diffdrive_teleop_ps3joy src/uos_diffdrive_teleop_ps3joy.cpp)
add_executable(uos_diffdrive_teleop_key src/uos_diffdrive_teleop_key.cpp)
add_executable(uos_diffdrive_teleop_wiimote src/uos_diffdrive_teleop_wiimote.cpp)

target_link_libraries(uos_diffdrive_teleop_cyborgevo ${catkin_LIBRARIES})
target_link_libraries(uos_diffdrive_teleop_ps3joy ${catkin_LIBRARIES})
target_link_libraries(uos_diffdrive_teleop_key ${catkin_LIBRARIES})
target_link_libraries(uos_diffdrive_teleop_wiimote ${catkin_LIBRARIES})


install(TARGETS
    uos_diffdrive_teleop_cyborgevo
    uos_diffdrive_teleop_ps3joy
    uos_diffdrive_teleop_key
    uos_diffdrive_teleop_wiimote
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
    launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
