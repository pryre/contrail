cmake_minimum_required(VERSION 2.8.3)
project(contrail_msgs)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
)

## Declare ROS messages and services
add_service_files(FILES
	SetTracking.srv
)

add_message_files(FILES
	CubicSpline.msg
	WaypointProgress.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES
	std_msgs
)

## Declare a catkin package
catkin_package(
#	DEPENDS
#	mantis_msgs_generate_messages
)
