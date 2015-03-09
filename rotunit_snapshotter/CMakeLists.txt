cmake_minimum_required(VERSION 2.4.6)

project(rotunit_snapshotter)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs laser_assembler)
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs laser_assembler
)

add_executable(rotunit_snapshotter src/rotunit_snapshotter.cpp)
target_link_libraries(rotunit_snapshotter ${catkin_LIBRARIES})
add_dependencies(rotunit_snapshotter ${catkin_EXPORTED_TARGETS})

install(TARGETS
  rotunit_snapshotter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
