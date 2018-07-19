#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <contrail_msgs/CubicSpline.h>
#include <contrail_msgs/SetTracking.h>
#include <contrail/ManagerParamsConfig.h>

#include <mavros_msgs/PositionTarget.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Splines>

#ifdef DO_NO_USE
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>

#include <iostream>

class SplineFunction {
public:
  SplineFunction(Eigen::VectorXd const &x_vec,
                 Eigen::VectorXd const &y_vec)
    : x_min(x_vec.minCoeff()),
      x_max(x_vec.maxCoeff()),
      // Spline fitting here. X values are scaled down to [0, 1] for this.
      spline_(Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(
                y_vec.transpose(),
                std::min<int>(x_vec.rows() - 1, 3),	//No more than cubic spline, but accept short vectors.
                scaled_values(x_vec)))
  { }

  double operator()(double x) const {
    // x values need to be scaled down in extraction as well.
    return spline_(scaled_value(x))(0);
  }

private:
  // Helpers to scale X values down to [0, 1]
  double scaled_value(double x) const {
    return (x - x_min) / (x_max - x_min);
  }

  Eigen::RowVectorXd scaled_values(Eigen::VectorXd const &x_vec) const {
    return x_vec.unaryExpr([this](double x) { return scaled_value(x); }).transpose();
  }

  double x_min;
  double x_max;

  // Spline of one-dimensional "points."
  Eigen::Spline<double, 1> spline_;
};

int main(int argc, char const* argv[])
{
  Eigen::VectorXd xvals(3);
  Eigen::VectorXd yvals(xvals.rows());

  xvals << 0, 15, 30;
  yvals << 0, 12, 17;

  SplineFunction s(xvals, yvals);

  std::cout << s(12.34) << std::endl;
}
#endif

class ContrailManager {
	public:
		enum TrackingRef {
			NONE,
			SPLINE,
			PATH,
			POSE
		};

	private:
		ros::NodeHandle& nhp_;

		ros::Subscriber sub_spline_;
		ros::Subscriber sub_path_;
		ros::Subscriber sub_pose_;

		ros::Publisher pub_discrete_progress_;	//Publishes the current path index when a discrete setpoint is reached
		ros::Publisher pub_spline_approx_;	//Publishes a approximate visualization of the calculated spline as feedback

		ros::ServiceServer srv_set_tracking_;
		dynamic_reconfigure::Server<contrail::ManagerParamsConfig> dyncfg_settings_;

		contrail_msgs::CubicSpline msg_spline_;
		nav_msgs::Path msg_path_;
		geometry_msgs::PoseStamped msg_pose_;

		bool param_fallback_to_pose_;
		bool pose_reached_;	//Check so that waypoint messages for pose are only output once
		int path_c_; //Counter for the current path step
		ros::Time dsp_reached_time_; //Check to keep track of if the current discrete setpoint has been reached already
		ros::Duration param_hold_duration_;
		double param_dsp_radius_;
		double param_dsp_yaw_;
		/*
		Eigen::Spline spline_px_;	//Position X
		Eigen::Spline spline_py_;	//Position Y
		Eigen::Spline spline_pz_;	//Position Z
		Eigen::Spline spline_ry_;	//Yaw
		*/
		typedef Eigen::Spline<double,5> Spline5d;
		typedef Eigen::Matrix<double,5,1> Vector5d;
		Spline5d spline_;	//[time,X;Y;Z;yaw]

		TrackingRef tracked_ref_;

	public:
		ContrailManager( ros::NodeHandle nh, const bool use_init_pose = false, const Eigen::Affine3d init_pose = Eigen::Affine3d::Identity() );

		~ContrailManager( void );

		//Allows control over the tracked reference
		TrackingRef get_reference_used( void );
		bool set_reference_used( TrackingRef state, const ros::Time t, const bool update_dsp_progress = false );

		//Returns the indicated reference can currently be tracked
		//has_reference() can be used to wait for an initial setpoint
		bool has_reference( const ros::Time t );
		bool has_spline_reference( const ros::Time t );
		bool has_path_reference( void );
		bool has_pose_reference( void );

		//Gets the current reference from the latest updated source
		//Returns true if the reference was successfully obtained
		bool get_reference( mavros_msgs::PositionTarget &ref, const ros::Time t, const geometry_msgs::Pose &p_c );

		//Gets the current reference from the spline source
		//Returns true if the reference was successfully obtained
		bool get_spline_reference( mavros_msgs::PositionTarget &ref, const ros::Time t );

		//Gets the current reference from the discrete path source
		//Returns true if the reference was successfully obtained
		bool get_discrete_path_reference( mavros_msgs::PositionTarget &ref, const ros::Time t, const geometry_msgs::Pose &p_c );

		//Gets the current reference from the discrete pose source
		//Returns true if the reference was successfully obtained
		bool get_discrete_pose_reference( mavros_msgs::PositionTarget &ref, const ros::Time t, const geometry_msgs::Pose &p_c );

		//Convinience versions of the get*reference();
		bool get_reference( mavros_msgs::PositionTarget &ref, const ros::Time t, const Eigen::Affine3d &g_c );
		bool get_discrete_path_reference( mavros_msgs::PositionTarget &ref, const ros::Time t, const Eigen::Affine3d &g_c );
		bool get_discrete_pose_reference( mavros_msgs::PositionTarget &ref, const ros::Time t, const Eigen::Affine3d &g_c );

		//Allows the trackings to be set directly rather than by callback
		bool set_spline_reference( const contrail_msgs::CubicSpline& spline );
		bool set_discrete_path_reference( const nav_msgs::Path& path );
		bool set_discrete_pose_reference( const geometry_msgs::PoseStamped& pose, const bool is_fallback = false );

	private:
		//ROS callbacks
		void callback_cfg_settings( contrail::ManagerParamsConfig &config, uint32_t level );

		void callback_spline( const contrail_msgs::CubicSpline::ConstPtr& msg_in );
		void callback_discrete_path( const nav_msgs::Path::ConstPtr& msg_in );
		void callback_discrete_pose( const geometry_msgs::PoseStamped::ConstPtr& msg_in );
		bool callback_set_tracking( contrail_msgs::SetTracking::Request &req, contrail_msgs::SetTracking::Response &res );

		void publish_waypoint_reached( const std::string frame_id, const ros::Time t, const uint32_t wp_c, const uint32_t wp_num );
		void publish_approx_spline( const std::string frame_id, const ros::Time& stamp, const ros::Time& ts, const ros::Time& te, const int steps, const Spline5d& s);

		//Returns true if the messages contain valid data
		bool check_msg_spline(const contrail_msgs::CubicSpline& spline, const ros::Time t );
		bool check_msg_path(const nav_msgs::Path& path );
		bool check_msg_pose(const geometry_msgs::PoseStamped& pose );

		//Returns true of the tracking point has been reached
		bool check_waypoint_reached( const Eigen::Vector3d& pos_s, const double yaw_s, const Eigen::Vector3d& pos_c, const double yaw_c );
		bool check_waypoint_complete( const ros::Time t );
		void reset_waypoint_timer( void );

		//Calculates the radial distance between two points
		double radial_dist( const Eigen::Vector3d a, const Eigen::Vector3d b );
		double rotation_dist( const double a, const double b );

		//Convinence functions for generating a position target from a pose
		mavros_msgs::PositionTarget target_from_pose( const ros::Time& time, const std::string& frame_id, const geometry_msgs::Pose& p );
		mavros_msgs::PositionTarget target_from_pose( const ros::Time& time, const std::string& frame_id, const Eigen::Affine3d& g );

		double yaw_from_quaternion( const geometry_msgs::Quaternion &q );
		double yaw_from_quaternion( const Eigen::Quaterniond &q );
		Eigen::Quaterniond quaternion_from_yaw( const double yaw );

		//Spline helper functions
		//Get the interpolated point from the spline,
		//	t should be the normalized time
		inline void get_spline_reference(Vector5d& p_interp, Vector5d& v_interp, const double t) const {
			// x values need to be scaled down in extraction as well.
			ROS_ASSERT_MSG((t >= 0.0) && (t <= 1.0), "Invalid time point given for spline interpolation (0.0 <= t <= 1.0)");

			//We only want the 0th and 1st-order derivatives
			const Eigen::Matrix<double,5,2> s = spline_.derivatives(t,1);
			p_interp = s.block<5,1>(0,0);
			v_interp = s.block<5,1>(0,1);
		}

		// Helpers to scale X values down to [0, 1]
		inline double normalize(double x, const double min, const double max) const {
			return (x - min) / (max - min);
		}

		/*
		template<typename Scalar>
		struct CwiseNormalizeOp {
		  CwiseNormalizeOp(const Scalar& min, const Scalar& max) : min_(min), max_(max) {}
		  const Scalar operator()(const Scalar& x) const { return (x - min_) / (max_ - min_); }
		  Scalar min_, max_;
		};
		inline Eigen::VectorXd normalize(const Eigen::VectorXd &x_vec) const {
			//save searching for the min and max constantly
			return x_vec.unaryExpr(CwiseNormalizeOp<double>(x_vec.minCoeff(), x_vec.maxCoeff()));
		}
		*/

		Eigen::Vector3d position_from_msg( const geometry_msgs::Point &p );
		Eigen::Quaterniond quaternion_from_msg( const geometry_msgs::Quaternion &q );
		Eigen::Affine3d affine_from_msg( const geometry_msgs::Pose &pose );

		geometry_msgs::Vector3 vector_from_eig( const Eigen::Vector3d &v );
		geometry_msgs::Point point_from_eig( const Eigen::Vector3d &p );
		geometry_msgs::Quaternion quaternion_from_eig( const Eigen::Quaterniond &q );
		geometry_msgs::Pose pose_from_eig( const Eigen::Affine3d &g );
};
