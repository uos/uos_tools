cmake_minimum_required(VERSION 2.8.3)
project(battery_guard)

find_package(catkin REQUIRED)

catkin_package(
	CATKIN_DEPENDS rospy sound_play
)

install(PROGRAMS
	scripts/battery_guard.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
	launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
