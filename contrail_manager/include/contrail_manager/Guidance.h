#pragma once

#include <ros/ros.h>

#include <contrail_manager/ContrailManager.h>

#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>

class Guidance {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;

		ros::Timer timer_;

		ros::Publisher pub_output_triplet_;
		ros::Publisher pub_output_position_;
		ros::Publisher pub_output_velocity_;

		ros::Subscriber sub_state_odometry_;

		Eigen::Affine3d current_g_;

		double param_rate_;
		bool param_do_feedback_;
		ros::Time odom_stamp_;

		ContrailManager ref_path_;

	public:
		Guidance( void );
		~Guidance( void );

	private:
		void callback_odom( const nav_msgs::Odometry::ConstPtr& msg_in );

		void callback_timer( const ros::TimerEvent& e );
};
