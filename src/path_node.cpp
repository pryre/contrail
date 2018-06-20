#include <ros/ros.h>
#include <contrail/path_generate.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "paths");
	PathGenerate pg;

	ros::spin();

	return 0;
}
