#include <ros/ros.h>
#include <contrail_manager/Guidance.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "guidance");
	Guidance pg;

	ros::spin();

	return 0;
}
