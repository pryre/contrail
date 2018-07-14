#include <ros/ros.h>

#include <contrail/path_generate.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Dense>

#include <vector>
#include <string>

PathGenerate::PathGenerate() :
	nhp_("~"),
	param_frame_id_("map"),
	param_res_(50),
	running_seq_(0) {

	//Setup publisher and get known parameters
	pub_path_ = nhp_.advertise<nav_msgs::Path>("path", 1, true);

	nhp_.param("frame_id", param_frame_id_, param_frame_id_);
	nhp_.param("complex_segment_resolution", param_res_, param_res_);

	//Give the node a moment to recieve a clock message (to allow it to work with simulated time)
	ros::Duration(0.2).sleep();
	ros::spinOnce();

	bool error = false;

	ROS_INFO("Loading path starting point");

	//Check that params were loaded correctly
	if( load_start_params() ) {
		add_pose(ros::Duration(0), pose_from_eigen(param_start_position_, param_start_orientation_));

		ROS_INFO("Generating path...");

		error = !generate_path();

		if(!error) {
			msg_out_.header.frame_id = param_frame_id_;
			msg_out_.header.stamp = ros::Time::now();

			pub_path_.publish(msg_out_);
		}
	} else {
		error = true;
		ROS_ERROR("Invalid starting parameters (position or direction)");
	}

	if(error)
		ros::shutdown();
}

PathGenerate::~PathGenerate() {
}

bool PathGenerate::load_start_params(void) {
	bool success = false;

	std::vector<double> start_p;
	double start_a = 0.0;
	double start_t = 0.0;
	double start_v = 0.0;

	if( nhp_.getParam("start/position", start_p) &&
		nhp_.getParam("start/alpha", start_a) ) {

		param_start_position_ = vector_from_doubles(start_p);
		param_start_direction_ = quaternion_from_yaw(start_a);

		if( nhp_.getParam("start/theta", start_t) ) {
			param_start_orientation_ = quaternion_from_yaw(start_t);
		} else {
			ROS_WARN("Using starting direction as starting orientation");
			param_start_orientation_ = param_start_direction_;
		}

		if( !nhp_.getParam("start/velocity", start_v) ) {
			ROS_WARN("Assuming path starts with zero velocity");
		}
		param_start_velocity_ = start_v;

		success = true;
	}

	return success;
}

bool PathGenerate::generate_path() {
	bool error = false;

	int i = 0;

	Eigen::Vector3d pc = param_start_position_;	//Current position
	Eigen::Quaterniond qcp = param_start_direction_;	//Current direction of the path
	Eigen::Quaterniond qcr = param_start_orientation_;	//Current direction of the robot
	double vc = param_start_velocity_;
	double velocity_last = param_start_velocity_;

	bool seg_present = false;

	//Loop through every segment available
	while( (!error) && nhp_.getParam("path/s" + std::to_string(i) + "/present", seg_present) ) {
		bool reticulate = false;

		double height = 0.0;
		nhp_.getParam("path/s" + std::to_string(i) + "/height", height);

		bool use_qo = false;
		Eigen::Quaterniond qo = Eigen::Quaterniond::Identity();
		double theta = 0.0;
		if( nhp_.getParam("path/s" + std::to_string(i) + "/theta", theta) ) {
			qo = quaternion_from_yaw(theta);
			use_qo = true;
		}

		double hold_time = 0.0;
		double length = 0.0;
		double velocity = 0.0;

		nhp_.getParam("path/s" + std::to_string(i) + "/hold_time", hold_time);
		nhp_.getParam("path/s" + std::to_string(i) + "/length", length);

		if(!nhp_.getParam("path/s" + std::to_string(i) + "/velocity", velocity) ) {
			ROS_DEBUG("Maintaining segment velocity (seg: %i)", i);
			velocity = velocity_last;
		}

		if(velocity < 0.0) {
			ROS_ERROR("Negative velocity specified (seg: %i)", i);
			error = true;
		}

		if(hold_time <= 0.0) {
			if( (velocity == 0.0 && velocity_last == 0.0) ) {
				ROS_ERROR("Line segment has no start or end velocity (seg: %i)", i);
				error = true;
			} else if(velocity != velocity_last) {
				//There is a change in velocity for this segment
				reticulate = true;
			}
		}

		double alpha = 0.0;
		double radius = 0.0;
		bool use_alpha = nhp_.getParam("path/s" + std::to_string(i) + "/alpha", alpha);
		bool use_radius = nhp_.getParam("path/s" + std::to_string(i) + "/radius", radius);

		if(use_radius && !use_alpha) {
			ROS_ERROR("Arc radius defined but rotation angle (alpha) defined (seg: %i)", i);
			error = true;
		} else if(use_alpha) {
			reticulate = true;
		}

		if(!error) {
			//Handle the basic cases first
			if(!reticulate) {
				ROS_DEBUG("Simple line segment (%i)", i);

				if(hold_time > 0.0) {
					qcr = (use_qo ? qo : qcr);	//Override the current heading if requested, else hold current heading

					add_pose(ros::Duration(hold_time), pose_from_eigen(pc, qcr) );
				} else if( (velocity > 0.0) ) {
					if( (length > 0.0) || (height != 0.0) ) {
						pc += qcp.toRotationMatrix()*Eigen::Vector3d(length, 0.0, 0.0) + Eigen::Vector3d(0.0, 0.0, height);
						double dist = Eigen::Vector3d(length, 0.0, height).norm();	//Segment distance
						qcr = (use_qo ? qo : qcr);	//Override the current heading if requested, else hold current heading

						add_pose(travel_time(dist, velocity), pose_from_eigen(pc, qcr) );
					} else {
						ROS_ERROR("Invalid length for simple line segment (seg: %i)", i);
						error = true;
					}
				} else {
					ROS_ERROR("Could not identify simple segment type to use (seg: %i)", i);
					ROS_ERROR("Params (#%i): hold_t=%0.4f; vel_last=%0.4f; vel_end=%0.4f; height=%0.4f; theta=%0.4f; length=%0.4f; alpha=%0.4f; radius=%0.4f",
										i, hold_time, velocity_last, velocity, height, theta, length, alpha, radius);

					error = true;
				}
			} else {
				ROS_DEBUG("Complex line segment (%i)", i);

				//Go through each subsegment
				double dalpha = alpha / param_res_;
				double dx = 0.0;
				double dz = height / param_res_;
				ros::Duration seg_time(0);

				//XXX: The complete segment should get the velocity to 1 "tick"
				//		away from the final value, otherwise the last segment will
				//		have a velocity of zero
				double dv = (velocity - velocity_last) / (param_res_ + 1);
				double vc_seg = velocity_last;

				for(int j=1; j<=param_res_; j++) {
					vc_seg += dv;

					//Figure out if it's a line, arc, or stopped turn
					if(hold_time > 0.0) {
						qcp *= quaternion_from_yaw(dalpha);
						qcr *= quaternion_from_yaw(dalpha);

						seg_time = ros::Duration(hold_time / param_res_);
					} else {	//Just to ensure correct check logic
						if(use_radius) { //Handle as an arc
							dx = std::fabs(radius*dalpha);

							pc += qcp.toRotationMatrix()*Eigen::Vector3d(dx, 0.0, 0.0) + Eigen::Vector3d(0.0, 0.0, dz);
							qcp *= quaternion_from_yaw(dalpha);
							qcr *= quaternion_from_yaw(dalpha);

							double dist = Eigen::Vector3d(dx, 0.0, dz).norm();	//Segment distance
							seg_time = travel_time(dist, vc_seg);
						} else if((length > 0.0) || (dz != 0.0) ) { //Else handle as a line
							dx = length / param_res_;
							pc += qcp.toRotationMatrix()*Eigen::Vector3d(dx, 0.0, 0.0) + Eigen::Vector3d(0.0, 0.0, dz);

							double dist = Eigen::Vector3d(dx, 0.0, dz).norm();	//Segment distance
							seg_time = travel_time(dist, vc_seg);

						} else {
							error = true;
						}
					}

					//Add in the segment if it is valid
					if(!error) {
						qcr = (use_qo ? qo : qcr);	//Override the current heading if requested, else hold current heading
						add_pose(seg_time, pose_from_eigen(pc, qcr) );
					} else {
						ROS_ERROR("Could not identify reticulated segment type to use (seg: %i)", i);
						ROS_ERROR("Params (#%i): hold_t=%0.4f; vel_last=%0.4f; vel_end=%0.4f; height=%0.4f; theta=%0.4f; length=%0.4f; alpha=%0.4f; radius=%0.4f",
											i, hold_time, velocity_last, velocity, height, theta, length, alpha, radius);

						break;
					}
				}
			}

			//Update running variables
			velocity_last = velocity;
		}

		i++;
	}

	if(!error) {
		ROS_INFO("Path generated with %i segments!", i);
	}

	if(i == 0)
		ROS_ERROR("Could not load any segments!");

	return (i > 0) & !error;
}

void PathGenerate::add_pose(const ros::Duration dt, const geometry_msgs::Pose pose) {
	geometry_msgs::PoseStamped pose_out;

	running_stamp_ += dt;

	pose_out.header.frame_id = param_frame_id_;
	pose_out.header.seq = running_seq_;
	pose_out.header.stamp = running_stamp_;

	pose_out.pose = pose;

	msg_out_.poses.push_back(pose_out);
	running_seq_++;
}

geometry_msgs::Pose PathGenerate::pose_from_eigen(const Eigen::Vector3d p, const Eigen::Quaterniond q) {
	geometry_msgs::Pose pose;
	Eigen::Quaterniond qc = q.normalized();

	pose.position.x = p.x();
	pose.position.y = p.y();
	pose.position.z = p.z();
	pose.orientation.w = qc.w();
	pose.orientation.x = qc.x();
	pose.orientation.y = qc.y();
	pose.orientation.z = qc.z();

	return pose;
}

Eigen::Vector3d PathGenerate::vector_from_doubles(std::vector<double> &a) {
	ROS_ASSERT_MSG( (a.size() == 3), "Vector3 size (%li) must be 3", a.size());

	return Eigen::Vector3d(a[0], a[1], a[2]);
}

Eigen::Quaterniond PathGenerate::quaternion_from_yaw( const double theta ) {
	return Eigen::Quaterniond( Eigen::AngleAxisd( theta, Eigen::Vector3d::UnitZ() ) );
}

ros::Duration PathGenerate::travel_time(const double len, const double vel) {
	return ros::Duration(len / vel);
}
