#include <ros/ros.h>

#include <contrail_manager/ContrailManager.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <contrail_manager/TrajectoryAction.h>
#include <contrail_manager/ManagerParamsConfig.h>
#include <contrail_spline_lib/quintic_spline_types.h>
#include <contrail_spline_lib/interpolated_quintic_spline.h>

#include <mavros_msgs/PositionTarget.h>

#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>
#include <math.h>


//=======================
// Public
//=======================

ContrailManager::ContrailManager( const ros::NodeHandle &nh, std::string frame_id, const bool is_ready ) :
	nhp_( nh, "contrail" ),
	param_frame_id_(frame_id),
	param_spline_approx_res_(0),
	param_end_position_accuracy_(0.0),
	param_end_yaw_accuracy_(0.0),
	param_ref_position_(false),
	param_ref_velocity_(false),
	param_ref_acceleration_(false),
	is_ready_(is_ready),
	spline_start_(0),
	spline_duration_(0),
	spline_in_progress_(false),
	wait_reached_end_(false),
	as_(nh, "contrail", false),
	dyncfg_settings_( nhp_ ) {

	dyncfg_settings_.setCallback(boost::bind(&ContrailManager::callback_cfg_settings, this, _1, _2));

	pub_spline_approx_ = nhp_.advertise<nav_msgs::Path>( "spline_approximation", 10, true );
	pub_spline_points_ = nhp_.advertise<nav_msgs::Path>( "spline_points", 10, true );
	pub_is_ready_ = nhp_.advertise<std_msgs::Bool>( "is_ready", 1, true );

	//Send out our first "is_ready" message
	allow_new_goals(is_ready_);

    as_.registerGoalCallback(boost::bind(&ContrailManager::callback_actionlib_goal, this));
    as_.registerPreemptCallback(boost::bind(&ContrailManager::callback_actionlib_preempt, this));
	as_.start();
}

ContrailManager::~ContrailManager( void ) {
}


void ContrailManager::set_frame_id( std::string frame_id ) {
	param_frame_id_ = frame_id;
}

bool ContrailManager::has_reference( const ros::Time t ) {
	return ( spline_start_ > ros::Time(0) );
}

void ContrailManager::clear_reference( void ) {
	spline_start_ = ros::Time(0);
	spline_in_progress_ = false;

	if( as_.isActive() )
		as_.setAborted();
}

void ContrailManager::allow_new_goals( const bool ready ) {
	is_ready_ = ready;

	std_msgs::Bool msg_out;
	msg_out.data = is_ready_;
	pub_is_ready_.publish(msg_out);
}

bool ContrailManager::is_allowing_new_goals( void ) {
	return is_ready_;
}

void ContrailManager::callback_actionlib_preempt(void) {
	ROS_INFO("Contrail: Preempted goal");
	as_.setPreempted();
	spline_in_progress_ = false;
	wait_reached_end_ = false;
}

void ContrailManager::callback_actionlib_goal(void) {
	//Check for a new goal
	if( as_.isNewGoalAvailable() )
		set_action_goal();

	// Check for a preempt
	if( as_.isPreemptRequested() )
		callback_actionlib_preempt();
}

void ContrailManager::set_action_goal( void ) {
	boost::shared_ptr<const contrail_manager::TrajectoryGoal> goal = as_.acceptNewGoal();

	if(is_ready_) {
		if( (goal->duration > ros::Duration(0) ) &&
			(goal->positions.size() >= 2) &&
			(goal->yaws.size() >= 2) ) {

			ros::Time tc = ros::Time::now();

			spline_in_progress_ = true;
			wait_reached_end_ = false;

			spline_start_ = ( goal->start == ros::Time(0) ) ? tc : goal->start;
			spline_duration_ = goal->duration;

			std::vector<double> positions_yaw = make_yaw_continuous( goal->yaws );

			Eigen::VectorXd vias_x = Eigen::VectorXd::Zero(goal->positions.size());
			Eigen::VectorXd vias_y = Eigen::VectorXd::Zero(goal->positions.size());
			Eigen::VectorXd vias_z = Eigen::VectorXd::Zero(goal->positions.size());
			Eigen::VectorXd vias_r = Eigen::VectorXd::Zero(positions_yaw.size());

			ROS_INFO( "Contrail: Creating trajectory [p:%u; y:%u]", (unsigned int)goal->positions.size(), (unsigned int)goal->yaws.size() );

			for(int i=0; i<goal->positions.size(); i++) {
				vias_x(i) = goal->positions[i].x;
				vias_y(i) = goal->positions[i].y;
				vias_z(i) = goal->positions[i].z;
			}

			for(int i=0; i<positions_yaw.size(); i++) {
				vias_r(i) = positions_yaw[i];
			}

			ROS_ASSERT_MSG( spline_x_.interpolate(vias_x), "Spline X interpolation failed!!!" );
			ROS_ASSERT_MSG( spline_y_.interpolate(vias_y), "Spline Y interpolation failed!!!" );
			ROS_ASSERT_MSG( spline_z_.interpolate(vias_z), "Spline Z interpolation failed!!!" );
			ROS_ASSERT_MSG( spline_r_.interpolate(vias_r), "Spline Yaw interpolation failed!!!" );

			spline_pos_start_ = vector_from_msg(goal->positions.front());
			spline_pos_end_ = vector_from_msg(goal->positions.back());
			spline_rot_start_ = goal->yaws.front();
			spline_rot_end_ = goal->yaws.back();

			publish_approx_spline(tc);
			publish_spline_points(tc, goal->positions, goal->yaws);

			ROS_DEBUG( "Contrail: creating position spline connecting %i points", (int)goal->positions.size() );
			ROS_DEBUG( "Contrail: creating rotation spline connecting %i points", (int)goal->yaws.size() );
		} else {
			clear_reference();

			ROS_ERROR( "Contrail: at least 2 positions/yaws must be specified (%i/%i), and duration must be >0 (%0.4f)", (int)goal->positions.size(), (int)goal->yaws.size(), goal->duration.toSec() );
		}
	} else {
		clear_reference();

		ROS_ERROR( "Contrail: not ready to accept goals (wait for controller to signal ready)" );
	}
}

bool ContrailManager::get_reference( mavros_msgs::PositionTarget &ref,
									 const ros::Time tc,
									 const geometry_msgs::Pose &pose ) {
	return get_reference( ref, tc, affine_from_msg(pose) );
}

bool ContrailManager::get_reference( mavros_msgs::PositionTarget &ref,
									 const ros::Time tc,
									 const Eigen::Affine3d &g_c ) {
	bool success = false;

	Eigen::Vector3d pos;
	Eigen::Vector3d vel;
	Eigen::Vector3d acc;
	double rpos;
	double rrate;

	if(	get_reference( pos, vel, acc, rpos, rrate, tc, g_c ) ) {
		ref.header.stamp = tc;
		ref.header.frame_id = param_frame_id_;

		ref.coordinate_frame = ref.FRAME_LOCAL_NED;
		ref.type_mask = 0;

		ref.position = point_from_eig(pos);
		ref.yaw = rpos;
		if(!param_ref_position_) {
			ref.type_mask |= ref.IGNORE_PX | ref.IGNORE_PY | ref.IGNORE_PZ | ref.IGNORE_YAW;
		}
		ref.velocity = vector_from_eig(vel);
		ref.yaw_rate = rrate;
		if(!param_ref_velocity_) {
			ref.type_mask |= ref.IGNORE_VX | ref.IGNORE_VY | ref.IGNORE_VZ | ref.IGNORE_YAW_RATE;
		}

		ref.acceleration_or_force = vector_from_eig(acc);
		if(!param_ref_acceleration_) {
			ref.type_mask |= ref.IGNORE_AFX | ref.IGNORE_AFY | ref.IGNORE_AFZ;
		} else if (param_ref_acceleration_ && !param_ref_position_ && !param_ref_velocity_) {
			// Edge-case for accel-only reference,
			// then we need to set yaw-rate to 0 at
			// the very least
			ref.yaw_rate = 0.0;
			ref.type_mask &= ~ref.IGNORE_YAW_RATE;
		}

		success = true;
	}

	return success;
}

bool ContrailManager::get_reference( Eigen::Vector3d &pos,
									 Eigen::Vector3d &vel,
									 Eigen::Vector3d &acc,
									 double &rpos,
									 double &rrate,
									 const ros::Time tc,
									 const Eigen::Affine3d &g_c ) {
	bool success = false;

	//If a valid input has been received
	if( has_reference( tc ) ) {
		//If in progress, calculate the lastest reference
		if( spline_in_progress_ ) {
			if( tc < spline_start_ ) {
				//Have no begun, stay at start position
				pos = spline_pos_start_;
				rpos = spline_rot_start_;
				vel = Eigen::Vector3d::Zero();
				acc = Eigen::Vector3d::Zero();
				rrate = 0.0;

				contrail_manager::TrajectoryFeedback feedback;
				feedback.progress = -1.0;
				feedback.position = vector_from_eig(pos);
				feedback.velocity = vector_from_eig(vel);
				feedback.acceleration = vector_from_eig(acc);
				feedback.yaw = rpos;
				feedback.yawrate = rrate;

				as_.publishFeedback(feedback);
			} else if( tc <= (spline_start_ + spline_duration_) ) {
				double t_norm = normalize((tc - spline_start_).toSec(), 0.0, spline_duration_.toSec());
				double npx = 0.0;
				double npy = 0.0;
				double npz = 0.0;
				double npr = 0.0;
				double nvx = 0.0;
				double nvy = 0.0;
				double nvz = 0.0;
				double nvr = 0.0;
				double nax = 0.0;
				double nay = 0.0;
				double naz = 0.0;
				double nar = 0.0;

				get_spline_reference(spline_x_, npx, nvx, nax, t_norm);
				get_spline_reference(spline_y_, npy, nvy, nay, t_norm);
				get_spline_reference(spline_z_, npz, nvz, naz, t_norm);
				get_spline_reference(spline_r_, npr, nvr, nar, t_norm);

				pos = Eigen::Vector3d(npx,npy,npz);
				rpos = npr;

				if(param_ref_velocity_) {
					vel = Eigen::Vector3d(nvx,nvy,nvz) / spline_duration_.toSec();
					rrate = nvr / spline_duration_.toSec();
				} else {
					acc = Eigen::Vector3d::Zero();
				}

				if(param_ref_acceleration_) {
					acc = Eigen::Vector3d(nax,nay,naz) / ( spline_duration_.toSec() * spline_duration_.toSec() );
				} else {
					acc = Eigen::Vector3d::Zero();
				}

				//nar is discarded

				contrail_manager::TrajectoryFeedback feedback;
				feedback.progress = t_norm;
				feedback.position = vector_from_eig(pos);
				feedback.velocity = vector_from_eig(vel);
				feedback.acceleration = vector_from_eig(acc);
				feedback.yaw = rpos;
				feedback.yawrate = rrate;

				as_.publishFeedback(feedback);
			} else {
				wait_reached_end_ = true;
				spline_in_progress_ = false;

				pos = spline_pos_end_;
				rpos = spline_rot_end_;
				vel = Eigen::Vector3d::Zero();
				acc = Eigen::Vector3d::Zero();
				rrate = 0.0;

				ROS_DEBUG( "Contrail: spline finished" );
			}

			output_pos_last_ = pos;
			output_rot_last_ = rpos;
		} else {
			pos = output_pos_last_;
			rpos = output_rot_last_;
			vel = Eigen::Vector3d::Zero();
			acc = Eigen::Vector3d::Zero();
			rrate = 0.0;
		}

		success = true;
	}

	check_end_reached(g_c);

	return success;
}



void ContrailManager::check_end_reached( const geometry_msgs::Pose &p_c ) {
	check_end_reached( affine_from_msg(p_c) );
}

void ContrailManager::check_end_reached( const Eigen::Affine3d &g_c ) {
	if(wait_reached_end_) {
		double yaw_c = yaw_from_quaternion( Eigen::Quaterniond(g_c.linear()) );
		if( check_endpoint_reached( spline_pos_end_,
									spline_rot_end_,
									g_c.translation(),
									yaw_c ) ) {

			contrail_manager::TrajectoryResult result;
			result.position_final = vector_from_eig( g_c.translation() );
			result.yaw_final = yaw_c;
			as_.setSucceeded(result);

			wait_reached_end_ = false;
			ROS_INFO( "Contrail: Trajectory complete" );
		}
	}
}

//=======================
// Private
//=======================

void ContrailManager::callback_cfg_settings( contrail_manager::ManagerParamsConfig &config, uint32_t level ) {
	param_end_position_accuracy_ = config.end_position_accuracy;
	param_end_yaw_accuracy_ = config.end_yaw_accuracy;
	param_spline_approx_res_ = config.spline_res_per_sec;
	param_ref_position_ = config.use_position_ref;
	param_ref_velocity_ = config.use_velocity_ref;
	param_ref_acceleration_ = config.use_acceleration_ref;
}

void ContrailManager::get_spline_reference( contrail_spline_lib::InterpolatedQuinticSpline& spline,
											double& pos,
											double& vel,
											double& acc,
											const double u) {
	// x values need to be scaled down in extraction as well.
	ROS_ASSERT_MSG((u >= 0.0) && (u <= 1.0), "Invalid time point given for spline interpolation (0.0 <= t <= 1.0)");
	ROS_ASSERT_MSG(spline.is_valid(), "Invalid spline request (not initialized?)");

	contrail_spline_lib::quintic_spline_point_t point = spline.lookup(u);
	pos = point.q;
	vel = point.qd;
	acc = point.qdd;
}

double ContrailManager::yaw_error_shortest_path(const double y_sp, const double y) {
	double ye = y_sp - y;

	while(fabs(ye) > M_PI)
		ye += (ye > 0.0) ? -2*M_PI : 2*M_PI;

	return ye;
}

std::vector<double> ContrailManager::make_yaw_continuous( const std::vector<double>& yaw ) {
	std::vector<double> cont_yaw;
	cont_yaw.reserve( yaw.size() );

	for(unsigned int i=0; i<yaw.size(); i++) {
		cont_yaw.push_back(yaw[i]);

		if ( i >= 1) {
			while(fabs(cont_yaw[i] - cont_yaw[i-1]) > M_PI) {
				if(cont_yaw[i] > cont_yaw[i-1]) {
					cont_yaw[i] -= 2*M_PI;
				} else {
					cont_yaw[i] += 2*M_PI;
				}
			}
		}
	}

	return cont_yaw;
}

void ContrailManager::publish_approx_spline( const ros::Time& stamp ) {
	nav_msgs::Path msg_out;

	msg_out.header.frame_id = param_frame_id_;
	msg_out.header.stamp = stamp;

	int num_points = spline_duration_.toSec() * param_spline_approx_res_;
	ros::Time t = spline_start_;
	ros::Duration dt = ros::Duration(spline_duration_.toSec() / num_points);
	int i = 0;
	for(int i=0; i<(num_points+1); i++) {
		//Reticulate the data of the current time
		double u = normalize(i,0,num_points);
		contrail_spline_lib::quintic_spline_point_t x = spline_x_.lookup(u);
		contrail_spline_lib::quintic_spline_point_t y = spline_y_.lookup(u);
		contrail_spline_lib::quintic_spline_point_t z = spline_z_.lookup(u);
		contrail_spline_lib::quintic_spline_point_t yaw = spline_r_.lookup(u);

		geometry_msgs::PoseStamped p;
		p.header.frame_id = msg_out.header.frame_id;
		p.header.stamp = t;
		p.header.seq = i;

		p.pose.position.x = x.q;
		p.pose.position.y = y.q;
		p.pose.position.z = z.q;
		p.pose.orientation = quaternion_from_eig(quaternion_from_yaw(yaw.q));

		msg_out.poses.push_back(p);

		t += dt;
	}

	pub_spline_approx_.publish(msg_out);
}

void ContrailManager::publish_spline_points( const ros::Time& stamp,
											 const std::vector<geometry_msgs::Vector3>& pos,
											 const std::vector<double>& yaw ) {
	nav_msgs::Path msg_out;

	msg_out.header.stamp = stamp;
	msg_out.header.frame_id = param_frame_id_;

	double dt = spline_duration_.toSec() / (pos.size() - 1);

	for(int i=0; i<pos.size(); i++) {
		geometry_msgs::PoseStamped p;
		p.header.frame_id = msg_out.header.frame_id;
		p.header.stamp = spline_start_ + ros::Duration(dt*i);
		p.header.seq = i;

		p.pose.position.x = pos[i].x;
		p.pose.position.y = pos[i].y;
		p.pose.position.z = pos[i].z;
		p.pose.orientation = quaternion_from_eig(quaternion_from_yaw(yaw[i]));

		msg_out.poses.push_back(p);
	}

	pub_spline_points_.publish(msg_out);
}

bool ContrailManager::check_endpoint_reached( const Eigen::Vector3d& pos_s, const double yaw_s, const Eigen::Vector3d& pos_c, const double yaw_c ) {
	return ( (radial_dist(pos_s, pos_c) < param_end_position_accuracy_) && (yaw_error_shortest_path(yaw_s, yaw_c) < param_end_yaw_accuracy_) );
}

double ContrailManager::radial_dist( const Eigen::Vector3d& a, const Eigen::Vector3d& b ) {
	return (a - b).norm();
}

/*
double ContrailManager::rotation_dist( const double a, const double b ) {
	double rad = std::fabs(a - b);
	//Adjust for +- short rotation
	return (rad > M_PI) ? rad - M_PI : rad;
}
*/

double ContrailManager::yaw_from_quaternion( const Eigen::Quaterniond &q ) {
	double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
	return std::atan2(siny, cosy);
}

Eigen::Quaterniond ContrailManager::quaternion_from_yaw( const double yaw ) {
	return Eigen::Quaterniond( Eigen::AngleAxisd( yaw, Eigen::Vector3d::UnitZ() ) );
}

Eigen::Vector3d ContrailManager::position_from_msg(const geometry_msgs::Point &p) {
	return Eigen::Vector3d(p.x, p.y, p.z);
}

Eigen::Quaterniond ContrailManager::quaternion_from_msg(const geometry_msgs::Quaternion &q) {
	return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
}

Eigen::Vector3d ContrailManager::vector_from_msg( const geometry_msgs::Vector3 &v ) {
	Eigen::Vector3d vec;

	vec.x() = v.x;
	vec.y() = v.y;
	vec.z() = v.z;

	return vec;
}

Eigen::Affine3d ContrailManager::affine_from_msg(const geometry_msgs::Pose &pose) {
	Eigen::Affine3d a;

	a.translation() << position_from_msg(pose.position);
	a.linear() << quaternion_from_msg(pose.orientation).toRotationMatrix();

	return a;
}

geometry_msgs::Vector3 ContrailManager::vector_from_eig(const Eigen::Vector3d &v) {
	geometry_msgs::Vector3 vec;

	vec.x = v.x();
	vec.y = v.y();
	vec.z = v.z();

	return vec;
}

geometry_msgs::Point ContrailManager::point_from_eig(const Eigen::Vector3d &p) {
	geometry_msgs::Point point;

	point.x = p.x();
	point.y = p.y();
	point.z = p.z();

	return point;
}

geometry_msgs::Quaternion ContrailManager::quaternion_from_eig(const Eigen::Quaterniond &q) {
	geometry_msgs::Quaternion quat;
	Eigen::Quaterniond qn = q.normalized();

	quat.w = qn.w();
	quat.x = qn.x();
	quat.y = qn.y();
	quat.z = qn.z();

	return quat;
}

geometry_msgs::Pose ContrailManager::pose_from_eig(const Eigen::Affine3d &g) {
	geometry_msgs::Pose pose;

	pose.position = point_from_eig(g.translation());
	pose.orientation = quaternion_from_eig(Eigen::Quaterniond(g.linear()));

	return pose;
}

