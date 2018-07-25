#include <ros/ros.h>

#include <contrail/ContrailManager.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <contrail_msgs/CubicSpline.h>
#include <contrail_msgs/PathProgress.h>
#include <contrail_msgs/SetTracking.h>
#include <contrail/ManagerParamsConfig.h>

#include <mavros_msgs/PositionTarget.h>

#include <eigen3/Eigen/Dense>
#include <string>
#include <math.h>


//=======================
// Public
//=======================

ContrailManager::ContrailManager( ros::NodeHandle nh, const bool use_init_pose, const Eigen::Affine3d init_pose ) :
	nhp_(nh),
	tracked_ref_(TrackingRef::NONE),
	param_fallback_to_pose_(false),
	pose_reached_(false),
	path_c_(0),
	dsp_reached_time_(0),
	param_hold_duration_(0),
	param_dsp_radius_(0.0),
	param_dsp_yaw_(0.0),
	dyncfg_settings_(ros::NodeHandle(nh, "contrail")) {

	dyncfg_settings_.setCallback(boost::bind(&ContrailManager::callback_cfg_settings, this, _1, _2));

	sub_spline_ = nhp_.subscribe<contrail_msgs::CubicSpline>( "reference/contrail/spline", 10, &ContrailManager::callback_spline, this );
	sub_path_ = nhp_.subscribe<nav_msgs::Path>( "reference/contrail/path", 10, &ContrailManager::callback_discrete_path, this );
	sub_pose_ = nhp_.subscribe<geometry_msgs::PoseStamped>( "reference/contrail/pose", 10, &ContrailManager::callback_discrete_pose, this );

	pub_discrete_progress_ = nhp_.advertise<contrail_msgs::PathProgress>( "feedback/contrail/discrete_progress", 10 );
	pub_spline_approx_ = nhp_.advertise<nav_msgs::Path>( "feedback/contrail/spline_approximation", 10 );
	pub_spline_points_ = nhp_.advertise<nav_msgs::Path>( "feedback/contrail/spline_points", 10 );

	srv_set_tracking_ = nhp_.advertiseService("contrail/set_tracking", &ContrailManager::callback_set_tracking, this);

	//Apply the initial pose setpoint if desired
	if(use_init_pose) {
		geometry_msgs::PoseStamped temp_pose;
		 //XXX: Just set a fake time for the init pose so it registers with has_pose_reference()
		temp_pose.header.stamp = ros::Time(1);
		temp_pose.pose = pose_from_eig(init_pose);
		set_discrete_pose_reference(temp_pose);
	}
}

ContrailManager::~ContrailManager( void ) {

}

ContrailManager::TrackingRef ContrailManager::get_reference_used( void ) {
	return tracked_ref_;
}

bool ContrailManager::set_reference_used( ContrailManager::TrackingRef state, const ros::Time t, bool update_dsp_progress ) {
	bool success = false;

	if(tracked_ref_ != state) {
		if( state == TrackingRef::NONE ) {
			success = true;
		} else if( ( state == TrackingRef::SPLINE ) && has_spline_reference(t) ) {
			ROS_INFO("[Contrail] Setting tracking to spline reference");
			success = true;
		} else if( ( state == TrackingRef::PATH ) && has_path_reference() ) {
			ROS_INFO("[Contrail] Setting tracking to path reference");
			update_dsp_progress = true;	//Always update discrete progress if switching

			success = true;
		} else if( ( state == TrackingRef::POSE ) && has_pose_reference() ) {
			ROS_INFO("[Contrail] Setting tracking to pose reference");
			if(!pose_reached_)
				update_dsp_progress = true;
			success = true;
		}

		if(success) {
			tracked_ref_ = state;

			//If we switched out of pose ref, then reset it
			if(tracked_ref_ != TrackingRef::POSE)
				pose_reached_ = false;
		}
	} else {
		//No need for a tracking change
		//But accept anyway
		success = true;
	}

	if(update_dsp_progress) {
		//Edge case for updating dsp output
		if(state == TrackingRef::PATH) {
			publish_waypoint_reached(msg_path_.header.frame_id, t, path_c_, msg_path_.poses.size());
		} else if(state == TrackingRef::POSE) {
			publish_waypoint_reached(msg_pose_.header.frame_id, t, 0, 1);
		}
	}

	return success;
}

bool ContrailManager::has_reference( const ros::Time t ) {
	return has_spline_reference(t) || has_path_reference() || has_pose_reference();
}

bool ContrailManager::has_spline_reference( const ros::Time t ) {
	return check_msg_spline(msg_spline_, t);
}

bool ContrailManager::has_path_reference( void ) {
	return check_msg_path(msg_path_);
}

bool ContrailManager::has_pose_reference( void ) {
	return check_msg_pose(msg_pose_);
}

bool ContrailManager::get_reference( mavros_msgs::PositionTarget &ref, const ros::Time t, const Eigen::Affine3d &g_c ) {
	bool success = false;

	if(get_reference_used() == TrackingRef::SPLINE) {
		success = get_spline_reference(ref, t);
	} else if(get_reference_used() == TrackingRef::PATH) {
		success = get_discrete_path_reference(ref, t, g_c);
	} else if(get_reference_used() == TrackingRef::POSE) {
		success = get_discrete_pose_reference(ref, t, g_c);
	}

	return success;
}

bool ContrailManager::get_spline_reference( mavros_msgs::PositionTarget &ref, const ros::Time t ) {
	bool success = true;

	if( has_spline_reference(t) ) {
		//Check time is within spline time, and handle cases where it's not
		if( t < msg_spline_.start_time ) {
			geometry_msgs::Pose hold;
			hold.position.x = msg_spline_.x.front();
			hold.position.y = msg_spline_.y.front();
			hold.position.z = msg_spline_.z.front();
			hold.orientation = quaternion_from_eig(quaternion_from_yaw(msg_spline_.yaw.front()));

			ref = target_from_pose(t, msg_spline_.header.frame_id, hold);
			ROS_INFO_THROTTLE(2.0, "[Contrail] Waiting for spline to start");
		} else if( t > ( msg_spline_.start_time + msg_spline_.duration ) ) {
			//We have finished the spline, but we haven't switched modes yet

			//Just track the last knot
			geometry_msgs::Pose hold;
			hold.position.x = msg_spline_.x.back();
			hold.position.y = msg_spline_.y.back();
			hold.position.z = msg_spline_.z.back();
			hold.orientation = quaternion_from_eig(quaternion_from_yaw(msg_spline_.yaw.back()));

			//Output a final pose
			ref = target_from_pose(t, msg_spline_.header.frame_id, hold);

			//Spline has finished
			if(param_fallback_to_pose_) {
				geometry_msgs::PoseStamped ps;
				ps.header.frame_id = msg_spline_.header.frame_id;
				ps.header.stamp = t;
				ps.pose = hold;

				//Should never fail, but just in case
				if(!set_discrete_pose_reference(ps, true) ) {
					ROS_ERROR("[Contrail] Error changing fallback tracking reference!");
				}
			} else {
				//Set back to no tracking
				if(!set_reference_used(TrackingRef::NONE, t)) {
					ROS_ERROR("[Contrail] Error changing tracking reference!");
				}
			}

			ROS_INFO("[Contrail] Finished spline reference");
		} else {
			// Spline tracking
			Eigen::Vector3d p_interp = Eigen::Vector3d::Zero();
			Eigen::Vector3d v_interp = Eigen::Vector3d::Zero();
			double yaw = 0.0;
			double yaw_rate = 0.0;

			//Get normalized time
			double t_norm = normalize((t - msg_spline_.start_time).toSec(), 0.0, msg_spline_.duration.toSec());

			//Interpolate spline
			get_spline_reference(p_interp, v_interp, yaw, yaw_rate, t_norm);

			ref.header.stamp = t;
			ref.header.frame_id = msg_spline_.header.frame_id;

			//Set up for a full trajectory reference
			ref.coordinate_frame = ref.FRAME_LOCAL_NED;
			ref.type_mask = ref.IGNORE_AFX | ref.IGNORE_AFY | ref.IGNORE_AFZ |
							ref.FORCE;

			ref.position.x = p_interp.x();
			ref.position.y = p_interp.y();
			ref.position.z = p_interp.z();
			ref.yaw = yaw;

			ref.velocity.x = v_interp.x() / msg_spline_.duration.toSec();
			ref.velocity.y = v_interp.y() / msg_spline_.duration.toSec();
			ref.velocity.z = v_interp.z() / msg_spline_.duration.toSec();
			ref.yaw_rate = yaw_rate / msg_spline_.duration.toSec();

			ref.acceleration_or_force.x = 0.0;
			ref.acceleration_or_force.y = 0.0;
			ref.acceleration_or_force.z = 0.0;
		}

		//ROS_INFO("spline reference pos: [%0.2f;%0.2f;%0.2f]", ref.position.x, ref.position.y, ref.position.z);
		//ROS_INFO("spline reference vel: [%0.2f;%0.2f;%0.2f]", ref.velocity.x, ref.velocity.y, ref.velocity.z);

		success = true;
	}

	return success;
}

bool ContrailManager::get_discrete_path_reference( mavros_msgs::PositionTarget &ref, const ros::Time t, const Eigen::Affine3d &g_c ) {
	bool success = false;

	if( has_path_reference() ) {
		//Prepare the tracked pose
		geometry_msgs::PoseStamped ps;
		ps.header.frame_id = msg_path_.header.frame_id;
		ps.header.stamp = t;

		//Keep tracking the poses as long as we haven't reached the end unexpectedly
		if(path_c_ < msg_path_.poses.size()) {
			//Just track the last pose
			ps.pose = msg_path_.poses[path_c_].pose;
			success = true;

			//Check all the logic for the next loop
			if( check_waypoint_reached(position_from_msg(msg_path_.poses[path_c_].pose.position),
									   yaw_from_quaternion(quaternion_from_msg(msg_path_.poses[path_c_].pose.orientation)),
									   g_c.translation(),
									   yaw_from_quaternion(Eigen::Quaterniond(g_c.linear()))) ) {
				//If reached the goal
				if( check_waypoint_complete(t) ) {
					if( path_c_ < msg_path_.poses.size() ) {
						//There are more waypoints left in the path
						path_c_++;
						reset_waypoint_timer();
					}

					//Update the path status
					publish_waypoint_reached(msg_path_.header.frame_id, t, path_c_, msg_path_.poses.size());

					//Handle the path being complete
					if( path_c_ >= msg_path_.poses.size() ) {
						if(param_fallback_to_pose_) {
							//Just track the last pose
							geometry_msgs::PoseStamped hold;
							hold.header = msg_path_.header;
							hold.pose = msg_path_.poses[msg_path_.poses.size()-1].pose;

							//Should never fail, but just in case
							if(!set_discrete_pose_reference(hold, true) ) {
								ROS_ERROR("[Contrail] Error changing fallback tracking reference!");
							}
						} else {
							//Set back to no tracking
							if(!set_reference_used(TrackingRef::NONE, t)) {
								ROS_ERROR("[Contrail] Error changing tracking reference!");
							}
						}

						ROS_INFO("[Contrail] Finished path reference");

						//Reset the path tracking
						path_c_ = 0;
						reset_waypoint_timer();
					}
				} //else we haven't stayed long enough, so keep tracking as usual
			} else {
				//We're outside the waypoint, so keep tracking
				reset_waypoint_timer();
			}
		}

		if(success) {
			ref = target_from_pose(t, msg_path_.header.frame_id, ps.pose);
		}
	}

	return success;
}

bool ContrailManager::get_discrete_pose_reference( mavros_msgs::PositionTarget &ref, const ros::Time t, const Eigen::Affine3d &g_c ) {
	bool success = true;

	if( has_pose_reference() ) {
		//Should we do waypoint checking?
		if(!pose_reached_) {
			//There is a valid pose reference
			if( check_waypoint_reached(position_from_msg(msg_pose_.pose.position),
									   yaw_from_quaternion(quaternion_from_msg(msg_pose_.pose.orientation)),
									   g_c.translation(),
									   yaw_from_quaternion(Eigen::Quaterniond(g_c.linear()))) ) {
				//If reached the goal
				if( check_waypoint_complete(t) ) {
					//Only send out the reached waypoint if reverting back to no tracking
					publish_waypoint_reached(msg_pose_.header.frame_id, t, 1, 1);

					if(!param_fallback_to_pose_) {
						//The waypoint is over, so reset to no tracking
						set_reference_used(TrackingRef::NONE, t);
					}

					ROS_INFO("[Contrail] Reached pose reference");

					//Set a flag to not bother checking waypoint later on
					pose_reached_ = true;
					reset_waypoint_timer();
				}
			} else {
				//We're outside the waypoint, so keep tracking
				reset_waypoint_timer();
			}
		}

		ref = target_from_pose(t, msg_pose_.header.frame_id, msg_pose_.pose);
		success = true;
	}

	return success;
}

bool ContrailManager::get_reference( mavros_msgs::PositionTarget &ref, const ros::Time t, const geometry_msgs::Pose &p_c ) {
	return get_reference(ref, t, affine_from_msg(p_c));
}

bool ContrailManager::get_discrete_path_reference( mavros_msgs::PositionTarget &ref, const ros::Time t, const geometry_msgs::Pose &p_c ) {
	return get_discrete_path_reference(ref, t, affine_from_msg(p_c));
}

bool ContrailManager::get_discrete_pose_reference( mavros_msgs::PositionTarget &ref, const ros::Time t, const geometry_msgs::Pose &p_c ) {
	return get_discrete_pose_reference(ref, t, affine_from_msg(p_c));
}

bool ContrailManager::set_spline_reference( const contrail_msgs::CubicSpline& spline ) {
	bool success = false;
	ros::Time tc = ros::Time::now();

	if( check_msg_spline(spline, tc) ) {
		//Input is valid, no do a check of the actual values
		ros::Time start_time = (spline.start_time == ros::Time(0)) ? tc : spline.start_time;

		if( ( start_time + spline.duration ) > tc ) {
			success = true;
		} else {
			ROS_WARN("[Contrail] Spline has already finished");
		}

		if(success) {
			msg_spline_ = spline;
			msg_spline_.start_time = start_time;

			make_yaw_continuous(msg_spline_.yaw);
			/*
			int num_knots = msg_spline_.x.size();
			Eigen::MatrixXd pknots(4,num_knots);
			Eigen::MatrixXd rknots(2,num_knots);

			for(int i=0; i<num_knots; i++) {
				pknots(0,i) = normalize(i, 0, num_knots);
				pknots(1,i) = msg_spline_.x[i];
				pknots(2,i) = msg_spline_.y[i];
				pknots(3,i) = msg_spline_.z[i];
				rknots(0,i) = normalize(i, 0, num_knots);
				rknots(1,i) = msg_spline_.yaw[i];
			}

			//Perform spline fitting
			//	t values must be normalized [0, 1]
			//	min is to ensure no more than cubic spline, but that we will also accept short vectors
			spline_p_ = Eigen::SplineFitting<Spline4d>::Interpolate( pknots, std::min<int>(pknots.cols() - 1, 3));
			spline_r_ = Eigen::SplineFitting<Spline2d>::Interpolate( rknots, std::min<int>(rknots.cols() - 1, 1));
			*/
			spline_x_ = tinyspline::Utils::interpolateCubic(&spline.x, 1);
			spline_y_ = tinyspline::Utils::interpolateCubic(&spline.y, 1);
			spline_z_ = tinyspline::Utils::interpolateCubic(&spline.z, 1);
			spline_yaw_ = tinyspline::Utils::interpolateCubic(&spline.yaw, 1);

			//spline_xd_ = spline_x_.derive();
			//spline_yd_ = spline_y_.derive();
			//spline_zd_ = spline_z_.derive();
			//spline_yawd_ = spline_yaw_.derive();
			/*
			for(int i=0; i<=100; i++) {
				double u = ((double)i)/100;
				double result = tspline_(u).result()[0];
				Eigen::Vector2d yaw = spline_r_(u);
				ROS_INFO("yaw_s(%0.2f): %0.2f", u, yaw[1]);
				ROS_INFO("yaw_t(%0.2f): %0.2f", u, result);
				//for(int j=0; j<=result.size(); j++)
				//	ROS_INFO("\t%0.2f", result[j]);
			}
			*/
			//	Also define the derrivatives at the start and end of the spline to be 0 (to ensure a smooth start/end trajectory
			/*
			Eigen::MatrixXd deriv = Eigen::MatrixXd::Zero(4,num_knots);
			Eigen::VectorXi di(num_knots);
			for(int i=0; i<num_knots; i++) {
				if( i == 0 ) {
					deriv.block<4,1>(0,i) = Eigen::Vector4d::Zero();
				} else if( i == (num_knots-1) ) {
					deriv.block<4,1>(0,i) = Eigen::Vector4d::Zero();
				} else {
					deriv.block<4,1>(0,i) = Eigen::Vector4d::Zero();// = (knots.block<4,1>(0,i+1) - knots.block<4,1>(0,i-1));
				}

				di(i) = i;
			}
			//di << 0, num_knots-1;

			spline_ = Eigen::SplineFitting<Spline4d>::InterpolateWithDerivatives( knots, deriv, di, std::min<int>(knots.cols() - 1, 3));
			*/

			publish_approx_spline( msg_spline_.header.frame_id,
								   msg_spline_.start_time,
								   msg_spline_.duration,
								   (int)(msg_spline_.duration*param_spline_approx_res_),
								   spline_x_,
								   spline_y_,
								   spline_z_,
								   spline_yaw_ );

		   publish_spline_points( msg_spline_ );

			if(!set_reference_used(TrackingRef::SPLINE, tc)) {
				ROS_ERROR("[Contrail] Error changing tracking reference!");
			}
		}
	}

	return success;
}

bool ContrailManager::set_discrete_path_reference( const nav_msgs::Path& path ) {
	bool success = false;
	ros::Time tc = ros::Time::now();

	if( check_msg_path(path) ) {
		msg_path_ = path;
		path_c_ = 0;
		reset_waypoint_timer();

		//Request a reference change, and ensure discrete progress update
		if(!set_reference_used(TrackingRef::PATH, tc, true)) {
			ROS_ERROR("[Contrail] Error changing tracking reference!");
		}

		success = true;
	}

	return success;
}

bool ContrailManager::set_discrete_pose_reference( const geometry_msgs::PoseStamped& pose, const bool is_fallback ) {
	bool success = false;
	ros::Time tc = ros::Time::now();

	if( check_msg_pose(pose) ) {
		msg_pose_ = pose;
		reset_waypoint_timer();
		pose_reached_ = is_fallback;

		//Request a reference change, and update discrete progress as long as it's not a fallback switch
		if(!set_reference_used(TrackingRef::POSE, tc, !is_fallback)) {
			ROS_ERROR("[Contrail] Error changing tracking reference!");
		}

		success = true;
	}

	return success;
}


//=======================
// Private
//=======================

void ContrailManager::callback_cfg_settings( contrail::ManagerParamsConfig &config, uint32_t level ) {
	param_fallback_to_pose_ = config.fallback_to_pose;
	param_hold_duration_ = ros::Duration(config.waypoint_hold_duration);
	param_dsp_radius_ = config.waypoint_radius;
	param_dsp_yaw_ = config.waypoint_yaw_accuracy;
	param_spline_approx_res_ = config.spline_res_per_sec;
}

void ContrailManager::callback_spline( const contrail_msgs::CubicSpline::ConstPtr& msg_in ) {
	if(!set_spline_reference(*msg_in)) {
		ROS_WARN("[Contrail] Spline reference invalid, ignoring");
	}
}

void ContrailManager::callback_discrete_path( const nav_msgs::Path::ConstPtr& msg_in ) {
	if(!set_discrete_path_reference(*msg_in)) {
		ROS_WARN("[Contrail] Path reference invalid, ignoring");
	}
}

void ContrailManager::callback_discrete_pose( const geometry_msgs::PoseStamped::ConstPtr& msg_in ) {
	if(!set_discrete_pose_reference(*msg_in)) {
		ROS_WARN("[Contrail] Pose reference invalid, ignoring");
	}
}

void ContrailManager::publish_waypoint_reached( const std::string frame_id,
												const ros::Time t,
												const uint32_t wp_c,
												const uint32_t wp_num ) {
	contrail_msgs::PathProgress msg_out;

	msg_out.header.frame_id = frame_id;
	msg_out.header.stamp = t;

	msg_out.current = wp_c;
	msg_out.progress = ((double)wp_c) / wp_num;

	pub_discrete_progress_.publish(msg_out);
}

void ContrailManager::publish_approx_spline( const std::string frame_id,
											 const ros::Time& stamp,
											 const ros::Duration& dur,
											 const int steps,
											 const tinyspline::BSpline& sx,
											 const tinyspline::BSpline& sy,
											 const tinyspline::BSpline& sz,
											 const tinyspline::BSpline& syaw ) {
	nav_msgs::Path msg_out;

	msg_out.header.frame_id = frame_id;
	msg_out.header.stamp = stamp;

	ros::Time t = stamp;
	ros::Duration dt = ros::Duration(dur.toSec() / steps);
	int i = 0;
	for(int i=0; i<(steps+1); i++) {
		//Reticulate the data of the current time
		double u = normalize(i,0,steps);
		std::vector<tinyspline::real> x = spline_x_(u).result();
		std::vector<tinyspline::real> y = spline_y_(u).result();
		std::vector<tinyspline::real> z = spline_z_(u).result();
		std::vector<tinyspline::real> yaw = spline_yaw_(u).result();

		//Eigen::Vector4d rp = sp(u);	//[X;Y;Z]
		//Eigen::Vector2d rr = sr(u);	//[yaw]

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

void ContrailManager::publish_spline_points( const contrail_msgs::CubicSpline& spline ) {
	nav_msgs::Path msg_out;

	msg_out.header = spline.header;

	double dt = spline.duration.toSec() / (spline.x.size() - 1);

	for(int i=0; i<spline.x.size(); i++) {
		geometry_msgs::PoseStamped p;
		p.header.frame_id = msg_out.header.frame_id;
		p.header.stamp = spline.start_time + ros::Duration(dt*i);
		p.header.seq = i;

		p.pose.position.x = spline.x[i];
		p.pose.position.y = spline.y[i];
		p.pose.position.z = spline.z[i];
		p.pose.orientation = quaternion_from_eig(quaternion_from_yaw(spline.yaw[i]));

		msg_out.poses.push_back(p);
	}

	pub_spline_points_.publish(msg_out);
}

bool ContrailManager::callback_set_tracking( contrail_msgs::SetTracking::Request  &req,
											 contrail_msgs::SetTracking::Response &res ) {
	if( req.tracking == req.TRACKING_NONE ) {
		res.success = set_reference_used(TrackingRef::NONE, ros::Time::now());
	} else if( req.tracking == req.TRACKING_SPLINE ) {
		res.success = set_reference_used(TrackingRef::SPLINE, ros::Time::now());
	} else if( req.tracking == req.TRACKING_PATH ) {
		res.success = set_reference_used(TrackingRef::PATH, ros::Time::now());
	} else if( req.tracking == req.TRACKING_POSE ) {
		res.success = set_reference_used(TrackingRef::POSE, ros::Time::now());
	} else {
		res.success = false;
	}

	return true;
}

bool ContrailManager::check_msg_spline(const contrail_msgs::CubicSpline& spline, const ros::Time t ) {
	//TODO: Check spline time is recent/not finished
	bool t_check = (spline.header.stamp != ros::Time(0)) && (spline.duration > ros::Duration(0));
	bool xvec_check = (spline.x.size() > 0);
	bool yvec_check = (spline.y.size() == spline.x.size());
	bool zvec_check = (spline.z.size() == spline.x.size());
	bool rvec_check = (spline.yaw.size() == spline.x.size());

	return (t_check && xvec_check && yvec_check && zvec_check && rvec_check);
}

bool ContrailManager::check_msg_path(const nav_msgs::Path& path ) {
	return ( (path.header.stamp > ros::Time(0)) && (path.poses.size() > 0) );
}

bool ContrailManager::check_msg_pose(const geometry_msgs::PoseStamped& pose ) {
	return pose.header.stamp > ros::Time(0);
}

bool ContrailManager::check_waypoint_reached( const Eigen::Vector3d& pos_s, const double yaw_s, const Eigen::Vector3d& pos_c, const double yaw_c ) {
	return ( (radial_dist(pos_s, pos_c) < param_dsp_radius_) && (rotation_dist(yaw_s, yaw_c) < param_dsp_yaw_) );
}

bool ContrailManager::check_waypoint_complete( const ros::Time t ) {
	bool reached = false;

	if( dsp_reached_time_ == ros::Time(0) ) {
		//if the waypoint was only just reached
		dsp_reached_time_ = t;
	} else if( (t - dsp_reached_time_) > param_hold_duration_) {
		//we had previously reached the waypoint, and we've been there long enough
		reached = true;
	}

	return reached;
}

void ContrailManager::reset_waypoint_timer( void ) {
	dsp_reached_time_ = ros::Time(0);
}

double ContrailManager::radial_dist( const Eigen::Vector3d a, const Eigen::Vector3d b ) {
	return (a - b).norm();
}

double ContrailManager::rotation_dist( const double a, const double b ) {
	double rad = std::fabs(a - b);
	//Adjust for +- short rotation
	return (rad > M_PI) ? rad - M_PI : rad;
}

mavros_msgs::PositionTarget ContrailManager::target_from_pose( const ros::Time& time, const std::string& frame_id, const geometry_msgs::Pose &p ) {
	mavros_msgs::PositionTarget msg_out;

	msg_out.header.stamp = time;
	msg_out.header.frame_id = frame_id;

	msg_out.coordinate_frame = msg_out.FRAME_LOCAL_NED;
	msg_out.type_mask = msg_out.IGNORE_VX | msg_out.IGNORE_VY | msg_out.IGNORE_VZ |
						msg_out.IGNORE_AFX | msg_out.IGNORE_AFY | msg_out.IGNORE_AFZ |
						msg_out.FORCE | msg_out.IGNORE_YAW_RATE;

	msg_out.position = p.position;
	msg_out.yaw = yaw_from_quaternion(quaternion_from_msg(p.orientation));

	msg_out.velocity.x = 0.0;
	msg_out.velocity.y = 0.0;
	msg_out.velocity.z = 0.0;
	msg_out.acceleration_or_force.x = 0.0;
	msg_out.acceleration_or_force.y = 0.0;
	msg_out.acceleration_or_force.z = 0.0;
	msg_out.yaw_rate = 0.0;

	return msg_out;
}

mavros_msgs::PositionTarget ContrailManager::target_from_pose( const ros::Time& time, const std::string& frame_id, const Eigen::Affine3d &g ) {
	return target_from_pose( time, frame_id, pose_from_eig(g) );
}

double ContrailManager::yaw_from_quaternion( const geometry_msgs::Quaternion &q ) {
	return yaw_from_quaternion(quaternion_from_msg(q));
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


