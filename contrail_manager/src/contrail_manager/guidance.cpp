#include <ros/ros.h>

#include <contrail_manager/Guidance.h>

#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <eigen3/Eigen/Dense>

Guidance::Guidance( void ) :
	nh_(),
	nhp_("~"),
	ref_path_(nhp_),
	param_do_feedback_(false),
	odom_stamp_(0),
	param_rate_(50.0) {

	current_g_ = Eigen::Affine3d::Identity();
	nhp_.param( "update_rate", param_rate_, param_rate_ );
	nhp_.param( "do_feedback", param_do_feedback_, param_do_feedback_ );

	sub_state_odometry_ = nhp_.subscribe<nav_msgs::Odometry>( "state/odom", 10, &Guidance::callback_odom, this );

	pub_output_triplet_ = nhp_.advertise<mavros_msgs::PositionTarget>( "command/triplet", 10 );
	pub_output_position_ = nhp_.advertise<geometry_msgs::PoseStamped>( "feedback/pose", 10 );
	pub_output_velocity_ = nhp_.advertise<geometry_msgs::TwistStamped>( "feedback/twist", 10 );

	timer_ = nhp_.createTimer( ros::Duration( 1.0 / param_rate_ ), &Guidance::callback_timer, this );

	ROS_INFO("Started guidance node, waiting for inputs");
}

Guidance::~Guidance( void ) {
}

void Guidance::callback_odom( const nav_msgs::Odometry::ConstPtr& msg_in ) {
	odom_stamp_ = msg_in->header.stamp;

	current_g_.translation() = Eigen::Vector3d(msg_in->pose.pose.position.x,
											   msg_in->pose.pose.position.y,
											   msg_in->pose.pose.position.z);

	current_g_.linear() = Eigen::Quaterniond(msg_in->pose.pose.orientation.w,
											 msg_in->pose.pose.orientation.x,
											 msg_in->pose.pose.orientation.y,
											 msg_in->pose.pose.orientation.z).normalized().toRotationMatrix();
}

void Guidance::callback_timer( const ros::TimerEvent& e ) {
	//Quick check to ensure our odom is relatively recent
	//  and that we have a reference
	if( ( (e.current_real - odom_stamp_) < ros::Duration(5/param_rate_) ) &&
		ref_path_.has_reference(e.current_real) ) {

		ROS_INFO_ONCE("Guidance outputting command!");

		mavros_msgs::PositionTarget traj;
		ref_path_.get_reference(traj, e.current_real, current_g_);

		pub_output_triplet_.publish(traj);

		if(param_do_feedback_) {
			geometry_msgs::PoseStamped p;
			geometry_msgs::TwistStamped t;

			p.header = traj.header;
			t.header = traj.header;

			p.pose.position = traj.position;
			Eigen::Quaterniond q(Eigen::AngleAxisd(traj.yaw, Eigen::Vector3d::UnitZ()));
			p.pose.orientation.w = q.w();
			p.pose.orientation.x = q.x();
			p.pose.orientation.y = q.y();
			p.pose.orientation.z = q.z();

			Eigen::Vector3d bv = current_g_.linear().inverse()*Eigen::Vector3d(traj.velocity.x, traj.velocity.y, traj.velocity.z);
			t.twist.linear.x = bv.x();
			t.twist.linear.y = bv.y();
			t.twist.linear.z = bv.z();
			t.twist.angular.x = 0.0;
			t.twist.angular.y = 0.0;
			t.twist.angular.z = traj.yaw_rate;

			pub_output_position_.publish(p);
			pub_output_velocity_.publish(t);
		}
	}
}

