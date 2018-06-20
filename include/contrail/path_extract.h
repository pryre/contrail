#pragma once

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <eigen3/Eigen/Dense>

class PathExtract {
	private:
		ros::NodeHandle *nh_;

		ros::Subscriber sub_path_;
		ros::Subscriber sub_fallback_;

		nav_msgs::Path p_c_;
		int path_hint_;
		bool have_path_;
		bool have_fallback_;

		Eigen::Affine3d latest_g_;

	public:
		PathExtract( ros::NodeHandle *nh, Eigen::Affine3d init_pose = Eigen::Affine3d::Identity() );

		~PathExtract( void );

		void set_fallback(const geometry_msgs::Pose &p);

		bool get_ref_state(Eigen::Affine3d &g_c, Eigen::Vector3d &v_c, const ros::Time tc);
		bool get_ref_state(geometry_msgs::Pose &pose, geometry_msgs::Vector3 &l_vel, const ros::Time tc);

		bool has_valid_path( void );
		bool has_valid_fallback( void );

	private:
		void callback_path(const nav_msgs::Path::ConstPtr& msg_in);
		void callback_fallback(const geometry_msgs::PoseStamped::ConstPtr& msg_in);

		void reset_hinting(void);
		void reset_path(void);

		Eigen::Vector3d position_from_msg(const geometry_msgs::Point &p);
		Eigen::Quaterniond quaternion_from_msg(const geometry_msgs::Quaternion &q);
		Eigen::Affine3d affine_from_msg(const geometry_msgs::Pose &pose);

		geometry_msgs::Vector3 vector_from_eig(const Eigen::Vector3d &v);
		geometry_msgs::Point point_from_eig(const Eigen::Vector3d &p);
		geometry_msgs::Quaternion quaternion_from_eig(const Eigen::Quaterniond &q);
		geometry_msgs::Pose pose_from_eig(const Eigen::Affine3d &g);

		Eigen::Vector3d vector_lerp(const Eigen::Vector3d a, const Eigen::Vector3d b, const double alpha);
};
