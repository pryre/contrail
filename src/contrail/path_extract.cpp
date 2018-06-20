#include <ros/ros.h>

#include <contrail/path_extract.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <eigen3/Eigen/Dense>

PathExtract::PathExtract( ros::NodeHandle *nh, Eigen::Affine3d init_pose ) :
	nh_(nh),
	have_path_(false),
	have_fallback_(false),
	path_hint_(1),
	latest_g_(init_pose) {

	sub_path_ = nh_->subscribe<nav_msgs::Path>( "reference/path", 10, &PathExtract::callback_path, this );
	sub_fallback_ = nh_->subscribe<geometry_msgs::PoseStamped>( "reference/path_fallback", 10, &PathExtract::callback_fallback, this );
}

PathExtract::~PathExtract( void ) {
}

bool PathExtract::has_valid_path( void ) {
	return have_path_;
}

bool PathExtract::has_valid_fallback( void ) {
	return have_fallback_;
}

void PathExtract::set_fallback(const geometry_msgs::Pose &pose) {
	latest_g_ = affine_from_msg(pose);
	have_fallback_ = true;
}

bool PathExtract::get_ref_state(geometry_msgs::Pose &pose, geometry_msgs::Vector3 &l_vel, const ros::Time tc) {
	Eigen::Affine3d g_c;
	Eigen::Vector3d v_c;

	bool success = get_ref_state(g_c, v_c, tc);

	pose = pose_from_eig(g_c);
	l_vel = vector_from_eig(v_c);

	return success;
}

bool PathExtract::get_ref_state(Eigen::Affine3d &g_c, Eigen::Vector3d &v_c, const ros::Time tc) {
	bool success = false;

	//Create temporary storage
	Eigen::Affine3d g_t = Eigen::Affine3d::Identity();
	Eigen::Vector3d v_t = Eigen::Vector3d::Zero();

	try {
		//If we have recieved a path message
		if(p_c_.header.stamp > ros::Time(0)) {
			ros::Duration duration_start = p_c_.poses.front().header.stamp - ros::Time(0);
			ros::Duration duration_end = p_c_.poses.back().header.stamp - ros::Time(0);

			ros::Time ts = p_c_.header.stamp + duration_start;
			ros::Time tf = p_c_.header.stamp + duration_end;
			ros::Duration td = duration_end - duration_start;

			//If the current time is within the path time, follow the path
			if((tc >= ts) && (tc < tf)) {
				//Check if we should use the path hint
				ros::Duration dth = p_c_.poses[path_hint_ - 1].header.stamp - ros::Time(0);
				int p = (tc > (p_c_.header.stamp + dth) ) ? path_hint_ : 1;

				//Find the next point in the path
				bool found_seg = false;

				while( ( !found_seg ) && ( p < p_c_.poses.size() ) ) {
					ros::Duration d_l = p_c_.poses[p - 1].header.stamp - ros::Time(0);
					ros::Duration d_n = p_c_.poses[p].header.stamp - ros::Time(0);

					if( (tc >= p_c_.header.stamp + d_l ) &&
						(tc < p_c_.header.stamp + d_n ) ) {
						//We have the right index
						found_seg = true;
					} else {
						//Keep looping
						p++;
					}
				}

				//Calculate the reference state as long as we found the segment
				if(found_seg) {
					//Get the last and next points
					Eigen::Affine3d g_l = affine_from_msg(p_c_.poses[p - 1].pose);
					Eigen::Affine3d g_n = affine_from_msg(p_c_.poses[p].pose);
					ros::Duration t_l = p_c_.poses[p - 1].header.stamp - ros::Time(0);
					ros::Duration t_n = p_c_.poses[p].header.stamp - ros::Time(0);
					ros::Time t_lt = p_c_.header.stamp + t_l;
					double dts = (t_n - t_l).toSec();	//Time to complete this segment
					double da = (tc - t_lt).toSec();	//Time to alpha
					double alpha = da / dts;

					//Position goal
					g_t.translation() << vector_lerp(g_l.translation(), g_n.translation(), alpha);
					Eigen::Quaterniond ql_sp(g_l.linear());
					Eigen::Quaterniond qc_sp = ql_sp.slerp(alpha, Eigen::Quaterniond(g_n.linear()));
					g_t.linear() << qc_sp.toRotationMatrix();

					//Velocity goal
					v_t = (g_n.translation() -  g_l.translation()) / dts;

					//Record the index we ues so we can start the time checks there next loop
					path_hint_ = p;

					success = true;
				}
			} else if(tc >= tf) {
				//else if our path is old
				//declare we no longer have a path
				ROS_INFO("Path complete!");
				reset_path();
			} else if(tc < ts) {
				//The path hasn't begun, so just wait
			} else {
				//Something weird is going on
				throw(std::runtime_error("Error syncronizing path"));
			}
		}
	} catch( std::runtime_error &e) {
		//May get errors if timestamp is malformed
		ROS_ERROR("Exception: [%s]", e.what());
		ROS_WARN("Invalidating current path!");

		reset_path();
	}

	if(success) {
		//Update latest setpoint in case we need to hold lastest position
		latest_g_ = g_t;
		have_fallback_ = true;

		g_c = g_t;
		v_c = v_t;
	} else {
		//Something went wrong, fall back to the last know safe values
		g_c = latest_g_;
		v_c = Eigen::Vector3d::Zero();

		reset_hinting();
	}

	return success;
}

void PathExtract::callback_path(const nav_msgs::Path::ConstPtr& msg_in) {
	//If there is at least 2 poses in the path
	if(msg_in->poses.size() > 1) {
		//If at least the very last timestamp is in the future, accept path
		if( ( msg_in->header.stamp + ( msg_in->poses.back().header.stamp - ros::Time(0) ) ) > ros::Time::now() ) {
			ROS_INFO("Recieved new path!");
			p_c_ = *msg_in;
			have_path_ = true;
			reset_hinting();
		} else {
			ROS_WARN("Rejecting path, timestamps are too old.");
		}
	} else {
		ROS_WARN("Rejecting path, must be at least 2 poses.");
	}
}

void PathExtract::callback_fallback(const geometry_msgs::PoseStamped::ConstPtr& msg_in) {
	//If there is at least 2 poses in the path
	set_fallback(msg_in->pose);
	ROS_INFO("Fallback pose set");
}

void PathExtract::reset_hinting(void) {
	path_hint_ = 1;	//Reset path hinting
}

void PathExtract::reset_path(void) {
	reset_hinting();
	p_c_.header.stamp = ros::Time(0);
	have_path_ = false;
}

Eigen::Vector3d PathExtract::position_from_msg(const geometry_msgs::Point &p) {
		return Eigen::Vector3d(p.x, p.y, p.z);
}

Eigen::Quaterniond PathExtract::quaternion_from_msg(const geometry_msgs::Quaternion &q) {
		return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
}

Eigen::Affine3d PathExtract::affine_from_msg(const geometry_msgs::Pose &pose) {
		Eigen::Affine3d a;

		a.translation() << position_from_msg(pose.position);
		a.linear() << quaternion_from_msg(pose.orientation).toRotationMatrix();

		return a;
}


geometry_msgs::Vector3 PathExtract::vector_from_eig(const Eigen::Vector3d &v) {
	geometry_msgs::Vector3 vec;

	vec.x = v.x();
	vec.y = v.y();
	vec.z = v.z();

	return vec;
}

geometry_msgs::Point PathExtract::point_from_eig(const Eigen::Vector3d &p) {
	geometry_msgs::Point point;

	point.x = p.x();
	point.y = p.y();
	point.z = p.z();

	return point;
}

geometry_msgs::Quaternion PathExtract::quaternion_from_eig(const Eigen::Quaterniond &q) {
	geometry_msgs::Quaternion quat;
	Eigen::Quaterniond qn = q.normalized();

	quat.w = qn.w();
	quat.x = qn.x();
	quat.y = qn.y();
	quat.z = qn.z();

	return quat;
}

geometry_msgs::Pose PathExtract::pose_from_eig(const Eigen::Affine3d &g) {
	geometry_msgs::Pose pose;

	pose.position = point_from_eig(g.translation());
	pose.orientation = quaternion_from_eig(Eigen::Quaterniond(g.linear()));

	return pose;
}

Eigen::Vector3d PathExtract::vector_lerp(const Eigen::Vector3d a, const Eigen::Vector3d b, const double alpha) {
  return ((1.0 - alpha) * a) + (alpha * b);
}
