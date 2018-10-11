#include <ros/ros.h>

#include <contrail/ContrailManager.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <contrail/TrajectoryAction.h>

#include <contrail/ManagerParamsConfig.h>

#include <mavros_msgs/PositionTarget.h>

#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>
#include <math.h>


//=======================
// Public
//=======================

ContrailManager::ContrailManager( ros::NodeHandle nhp, std::string frame_id ) :
	nhp_(nhp),
	param_frame_id_(frame_id),
	param_spline_approx_res_(0),
	param_end_position_accuracy_(0.0),
	param_end_yaw_accuracy_(0.0),
	spline_start_(0),
	spline_duration_(0),
	spline_in_progress_(false),
	wait_reached_end_(false),
	use_dirty_derivative_(false),
	as_(nhp, "contrail", false),
	dyncfg_settings_(ros::NodeHandle(nhp, "contrail")) {

	dyncfg_settings_.setCallback(boost::bind(&ContrailManager::callback_cfg_settings, this, _1, _2));

	pub_spline_approx_ = nhp_.advertise<nav_msgs::Path>( "contrail/spline_approximation", 10 );
	pub_spline_points_ = nhp_.advertise<nav_msgs::Path>( "contrail/spline_points", 10 );

	timer_ = nhp_.createTimer(ros::Duration(1.0/50.0), &ContrailManager::callback_timer, this );
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

void ContrailManager::callback_timer(const ros::TimerEvent& e) {
	//Check for a new goal
	if( as_.isNewGoalAvailable() ) {
		set_action_goal();
	}

	// Check that preempt has not been requested by the client
	if( as_.isPreemptRequested() ) {
		ROS_INFO("Contrail: Preempted");
		as_.setPreempted();
		spline_in_progress_ = false;
		wait_reached_end_ = false;
	}
}

void ContrailManager::set_action_goal( void ) {
	boost::shared_ptr<const contrail::TrajectoryGoal> goal = as_.acceptNewGoal();

	if( (goal->duration > ros::Duration(0) ) &&
		(goal->positions.size() >= 2) &&
		(goal->yaws.size() >= 2) ) {

		ros::Time tc = ros::Time::now();

		spline_in_progress_ = true;
		wait_reached_end_ = false;

		spline_start_ = ( goal->start == ros::Time(0) ) ? tc : goal->start;
		spline_duration_ = goal->duration;

		std::vector<double>positions_x;
		std::vector<double>positions_y;
		std::vector<double>positions_z;

		for(int i=0; i<goal->positions.size(); i++) {
			positions_x.push_back(goal->positions[i].x);
			positions_y.push_back(goal->positions[i].y);
			positions_z.push_back(goal->positions[i].z);
		}

		std::vector<double>positions_yaw = goal->yaws;
		make_yaw_continuous(positions_yaw);

		spline_x_ = tinyspline::Utils::interpolateCubic(&positions_x, 1);
		spline_y_ = tinyspline::Utils::interpolateCubic(&positions_y, 1);
		spline_z_ = tinyspline::Utils::interpolateCubic(&positions_z, 1);
		spline_r_ = tinyspline::Utils::interpolateCubic(&positions_yaw, 1);

		//Smooth out control points to give a nicer fit
		std::vector<tinyspline::real> ctrlp_x = spline_x_.controlPoints();
		std::vector<tinyspline::real> ctrlp_y = spline_y_.controlPoints();
		std::vector<tinyspline::real> ctrlp_z = spline_z_.controlPoints();
		std::vector<tinyspline::real> ctrlp_r = spline_r_.controlPoints();
		ROS_ASSERT_MSG( ctrlp_x.size() >= 4, "Number of pos_x control points is <4 (%i)", (int)ctrlp_x.size());
		ROS_ASSERT_MSG( ctrlp_y.size() >= 4, "Number of pos_y control points is <4 (%i)", (int)ctrlp_y.size());
		ROS_ASSERT_MSG( ctrlp_z.size() >= 4, "Number of pos_z control points is <4 (%i)", (int)ctrlp_z.size());
		ROS_ASSERT_MSG( ctrlp_r.size() >= 4, "Number of yaw control points is <4 (%i)", (int)ctrlp_r.size());

		ctrlp_x.at(1) = ctrlp_x.front();
		ctrlp_x.at(ctrlp_x.size() - 2) = ctrlp_x.back();
		spline_x_.setControlPoints(ctrlp_x);
		ctrlp_y.at(1) = ctrlp_y.front();
		ctrlp_y.at(ctrlp_y.size() - 2) = ctrlp_y.back();
		spline_y_.setControlPoints(ctrlp_y);
		ctrlp_z.at(1) = ctrlp_z.front();
		ctrlp_z.at(ctrlp_z.size() - 2) = ctrlp_z.back();
		spline_z_.setControlPoints(ctrlp_z);
		ctrlp_r.at(1) = ctrlp_r.front();
		ctrlp_r.at(ctrlp_r.size() - 2) = ctrlp_r.back();
		spline_r_.setControlPoints(ctrlp_r);

		try {
			spline_xd_ = spline_x_.derive();
			spline_yd_ = spline_y_.derive();
			spline_zd_ = spline_z_.derive();
			spline_rd_ = spline_r_.derive();

			use_dirty_derivative_ = false;
		}
		catch(std::runtime_error) {
			use_dirty_derivative_ = true;
		}

		spline_pos_start_ = vector_from_msg(goal->positions.front());
		spline_pos_end_ = vector_from_msg(goal->positions.back());
		spline_rot_start_ = goal->yaws.front();
		spline_rot_end_ = goal->yaws.back();

		publish_approx_spline(tc);
		publish_spline_points(tc, goal->positions, goal->yaws);

		ROS_INFO( "Contrail: creating position spline connecting %i points", (int)goal->positions.size() );
		ROS_INFO( "Contrail: creating rotation spline connecting %i points", (int)goal->yaws.size() );
	} else {
		spline_in_progress_ = false;
		as_.setAborted();

		ROS_ERROR( "Contrail: at least 2 positions/yaws must be specified (%i/%i), and duration must be >0 (%0.4f)", (int)goal->positions.size(), (int)goal->yaws.size(), goal->duration.toSec() );
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
	double rpos;
	double rrate;

	if(	get_reference( pos, vel, rpos, rrate, tc ) ) {
		ref.header.stamp = tc;
		ref.header.frame_id = param_frame_id_;

		ref.coordinate_frame = ref.FRAME_LOCAL_NED;
		ref.type_mask =	ref.IGNORE_AFX | ref.IGNORE_AFY | ref.IGNORE_AFZ | ref.FORCE;

		ref.position = point_from_eig(pos);
		ref.yaw = rpos;

		ref.velocity = vector_from_eig(vel);
		ref.yaw_rate = rrate;

		ref.acceleration_or_force.x = 0.0;
		ref.acceleration_or_force.y = 0.0;
		ref.acceleration_or_force.z = 0.0;

		success = true;
	}

	check_end_reached(g_c);

	return success;
}

bool ContrailManager::get_reference( Eigen::Vector3d &pos,
									 Eigen::Vector3d &vel,
									 double &rpos,
									 double &rrate,
									 const ros::Time tc ) {
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
				rrate = 0.0;

				contrail::TrajectoryFeedback feedback;
				feedback.progress = -1.0;
				feedback.position = vector_from_eig(pos);
				feedback.velocity = vector_from_eig(vel);
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

				get_spline_reference(spline_x_, spline_xd_, npx, nvx, t_norm);
				get_spline_reference(spline_y_, spline_yd_, npy, nvy, t_norm);
				get_spline_reference(spline_z_, spline_zd_, npz, nvz, t_norm);
				get_spline_reference(spline_r_, spline_rd_, npr, nvr, t_norm);

				pos = Eigen::Vector3d(npx,npy,npz);
				vel = Eigen::Vector3d(nvx,nvy,nvz) / spline_duration_.toSec();
				rpos = npr;
				rrate = nvr / spline_duration_.toSec();

				contrail::TrajectoryFeedback feedback;
				feedback.progress = t_norm;
				feedback.position = vector_from_eig(pos);
				feedback.velocity = vector_from_eig(vel);
				feedback.yaw = rpos;
				feedback.yawrate = rrate;

				as_.publishFeedback(feedback);
			} else {
				wait_reached_end_ = true;
				spline_in_progress_ = false;

				pos = spline_pos_end_;
				rpos = spline_rot_end_;
				vel = Eigen::Vector3d::Zero();
				rrate = 0.0;

				ROS_INFO( "Contrail: spline finished" );
			}

			output_pos_last_ = pos;
			output_rot_last_ = rpos;
		} else {
			pos = output_pos_last_;
			rpos = output_rot_last_;
			vel = Eigen::Vector3d::Zero();
			rrate = 0.0;
		}

		success = true;
	}

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

			contrail::TrajectoryResult result;
			result.position_final = vector_from_eig( g_c.translation() );
			result.yaw_final = yaw_c;
			as_.setSucceeded(result);

			wait_reached_end_ = false;
			ROS_INFO( "Contrail: action finished" );
		}
	}
}

//=======================
// Private
//=======================

void ContrailManager::callback_cfg_settings( contrail::ManagerParamsConfig &config, uint32_t level ) {
	param_end_position_accuracy_ = config.end_position_accuracy;
	param_end_yaw_accuracy_ = config.end_yaw_accuracy;
	param_spline_approx_res_ = config.spline_res_per_sec;
}

void ContrailManager::get_spline_reference(tinyspline::BSpline& spline,
											tinyspline::BSpline& splined,
											double& pos,
											double& vel,
											const double u) {
	// x values need to be scaled down in extraction as well.
	ROS_ASSERT_MSG((u >= 0.0) && (u <= 1.0), "Invalid time point given for spline interpolation (0.0 <= t <= 1.0)");

	std::vector<tinyspline::real> vp = spline(u).result();
	pos = vp[0];

	if(!use_dirty_derivative_) {
		std::vector<tinyspline::real> vv = splined(u).result();
		vel = vv[0];
	} else {
		//XXX: Manually derrive over a short period as proper derivative can't be calculated using this library
		double dt = 0.02;
		//Shorten time to ensure that 0.0<=u<=1.0 is preserved
		double ul = u - dt;
		double uh = u + dt;
		ul = (ul >= 0.0) ? ul : 0.0;
		uh = (uh <= 1.0) ? uh : 1.0;

		std::vector<tinyspline::real> vdl = spline(ul).result();
		std::vector<tinyspline::real> vdh = spline(uh).result();

		vel = (vdh[0] - vdl[0]) / (2*dt);
	}
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
		std::vector<tinyspline::real> x = spline_x_(u).result();
		std::vector<tinyspline::real> y = spline_y_(u).result();
		std::vector<tinyspline::real> z = spline_z_(u).result();
		std::vector<tinyspline::real> yaw = spline_r_(u).result();

		geometry_msgs::PoseStamped p;
		p.header.frame_id = msg_out.header.frame_id;
		p.header.stamp = t;
		p.header.seq = i;

		p.pose.position.x = x[0];
		p.pose.position.y = y[0];
		p.pose.position.z = z[0];
		p.pose.orientation = quaternion_from_eig(quaternion_from_yaw(yaw[0]));

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
	return ( (radial_dist(pos_s, pos_c) < param_end_position_accuracy_) && (rotation_dist(yaw_s, yaw_c) < param_end_yaw_accuracy_) );
}

double ContrailManager::radial_dist( const Eigen::Vector3d& a, const Eigen::Vector3d& b ) {
	return (a - b).norm();
}

double ContrailManager::rotation_dist( const double a, const double b ) {
	double rad = std::fabs(a - b);
	//Adjust for +- short rotation
	return (rad > M_PI) ? rad - M_PI : rad;
}

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

