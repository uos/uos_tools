cmake_minimum_required(VERSION 3.8)
project(uos_diffdrive_teleop)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(
    include
)

# add_executable(uos_diffdrive_teleop_cyborgevo 
#     src/uos_diffdrive_teleop_cyborgevo.cpp
# )
# ament_target_dependencies(uos_diffdrive_teleop_cyborgevo
#     rclcpp
#     sensor_msgs
#     geometry_msgs
# )

# add_executable(uos_diffdrive_teleop_ps3joy src/uos_diffdrive_teleop.cpp src/uos_diffdrive_teleop_ps3joy.cpp)
# target_link_libraries(uos_diffdrive_teleop_ps3joy ${catkin_LIBRARIES})
# add_dependencies(uos_diffdrive_teleop_ps3joy ${catkin_EXPORTED_TARGETS})

add_executable(uos_diffdrive_teleop_key 
    src/uos_diffdrive_teleop.cpp 
    src/uos_diffdrive_teleop_key.cpp)

ament_target_dependencies(uos_diffdrive_teleop_key
    rclcpp
    sensor_msgs
    geometry_msgs
)

# add_executable(uos_diffdrive_teleop_wiimote src/uos_diffdrive_teleop_wiimote.cpp)
# target_link_libraries(uos_diffdrive_teleop_wiimote ${catkin_LIBRARIES})
# add_dependencies(uos_diffdrive_teleop_wiimote ${catkin_EXPORTED_TARGETS})

install(TARGETS
    # uos_diffdrive_teleop_cyborgevo
    # uos_diffdrive_teleop_ps3joy
    uos_diffdrive_teleop_key
    # uos_diffdrive_teleop_wiimote
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()