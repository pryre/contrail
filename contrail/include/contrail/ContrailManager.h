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

#include <contrail/tinysplinecpp.h>

#include <mavros_msgs/PositionTarget.h>

#include <eigen3/Eigen/Dense>

#include <vector>
#include <math.h>

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
		ros::Publisher pub_spline_points_;	//Publishes a path representing the interpolated spline points

		ros::ServiceServer srv_set_tracking_;
		dynamic_reconfigure::Server<contrail::ManagerParamsConfig> dyncfg_settings_;

		contrail_msgs::CubicSpline msg_spline_;
		nav_msgs::Path msg_path_;
		geometry_msgs::PoseStamped msg_pose_;

		bool param_fallback_to_pose_;
		bool pose_reached_;	//Check so that waypoint messages for pose are only output once
		int path_c_; //Counter for the current path step
		double param_nominal_lvel_;
		double param_nominal_rvel_;
		double param_dsp_radius_;
		double param_dsp_yaw_;
		double param_spline_approx_res_;
		/*
		Eigen::Spline spline_px_;	//Position X
		Eigen::Spline spline_py_;	//Position Y
		Eigen::Spline spline_pz_;	//Position Z
		Eigen::Spline spline_ry_;	//Yaw
		*/
		//typedef Eigen::Spline<double,2> Spline2d;
		//typedef Eigen::Spline<double,4> Spline4d;
		//Spline4d spline_p_;	//[time;X;Y;Z]
		//Spline2d spline_r_;	//[time;yaw]

		tinyspline::BSpline spline_x_;
		tinyspline::BSpline spline_y_;
		tinyspline::BSpline spline_z_;
		tinyspline::BSpline spline_yaw_;

		tinyspline::BSpline dsp_spline_x_;
		tinyspline::BSpline dsp_spline_y_;
		tinyspline::BSpline dsp_spline_z_;
		tinyspline::BSpline dsp_spline_yaw_;
		ros::Time dsp_spline_start_;
		ros::Duration dsp_spline_dur_;
		/*
		//XXX: Manually derrive over a short period as proper derivative can't be calculated using this library
		tinyspline::BSpline spline_xd_;
		tinyspline::BSpline spline_yd_;
		tinyspline::BSpline spline_zd_;
		tinyspline::BSpline spline_yawd_;
		*/

		TrackingRef tracked_ref_;

	public:
		ContrailManager( ros::NodeHandle nh,
						 const bool use_init_pose = false,
						 const Eigen::Affine3d init_pose = Eigen::Affine3d::Identity() );

		~ContrailManager( void );

		//Allows control over the tracked reference
		TrackingRef get_reference_used( void );
		bool set_reference_used( TrackingRef state,
								 const ros::Time t,
								 const bool update_dsp_progress = false );

		//Returns the indicated reference can currently be tracked
		//has_reference() can be used to wait for an initial setpoint
		bool has_reference( const ros::Time t );
		bool has_spline_reference( const ros::Time t );
		bool has_path_reference( void );
		bool has_pose_reference( void );

		//Gets the current reference from the latest updated source
		//Returns true if the reference was successfully obtained
		bool get_reference( mavros_msgs::PositionTarget &ref,
							const ros::Time t,
							const geometry_msgs::Pose &p_c );

		//Gets the current reference from the spline source
		//Returns true if the reference was successfully obtained
		bool get_spline_reference( mavros_msgs::PositionTarget &ref,
								   const ros::Time t );

		//Gets the current reference from the discrete path source
		//Returns true if the reference was successfully obtained
		bool get_discrete_path_reference( mavros_msgs::PositionTarget &ref,
										  const ros::Time t,
										  const geometry_msgs::Pose &p_c );

		//Gets the current reference from the discrete pose source
		//Returns true if the reference was successfully obtained
		bool get_discrete_pose_reference( mavros_msgs::PositionTarget &ref,
										  const ros::Time t,
										  const geometry_msgs::Pose &p_c );

		//Convinience versions of the get*reference();
		bool get_reference( mavros_msgs::PositionTarget &ref,
							const ros::Time t,
							const Eigen::Affine3d &g_c );
		bool get_discrete_path_reference( mavros_msgs::PositionTarget &ref,
										  const ros::Time t,
										  const Eigen::Affine3d &g_c );
		bool get_discrete_pose_reference( mavros_msgs::PositionTarget &ref,
										  const ros::Time t,
										  const Eigen::Affine3d &g_c );

		//Allows the trackings to be set directly rather than by callback
		bool set_spline_reference( const contrail_msgs::CubicSpline& spline );
		bool set_discrete_path_reference( const nav_msgs::Path& path );
		bool set_discrete_pose_reference( const geometry_msgs::PoseStamped& pose,
										  const bool is_fallback = false );

	private:
		//ROS callbacks
		void callback_cfg_settings( contrail::ManagerParamsConfig &config, uint32_t level );

		void callback_spline( const contrail_msgs::CubicSpline::ConstPtr& msg_in );
		void callback_discrete_path( const nav_msgs::Path::ConstPtr& msg_in );
		void callback_discrete_pose( const geometry_msgs::PoseStamped::ConstPtr& msg_in );
		bool callback_set_tracking( contrail_msgs::SetTracking::Request &req,
									contrail_msgs::SetTracking::Response &res );

		void publish_waypoint_reached( const std::string frame_id,
									   const ros::Time t,
									   const uint32_t wp_c,
									   const uint32_t wp_num );
		void publish_approx_spline( const std::string frame_id,
									const ros::Time& stamp,
									const ros::Duration& dur,
									const int steps,
									const tinyspline::BSpline& sx,
									const tinyspline::BSpline& sy,
									const tinyspline::BSpline& sz,
									const tinyspline::BSpline& syaw );
		void publish_spline_points( const contrail_msgs::CubicSpline& spline );

		//Returns true if the messages contain valid data
		bool check_msg_spline(const contrail_msgs::CubicSpline& spline, const ros::Time t );
		bool check_msg_path(const nav_msgs::Path& path );
		bool check_msg_pose(const geometry_msgs::PoseStamped& pose );

		//Returns true of the tracking point has been reached
		bool check_waypoint_reached( const Eigen::Vector3d& pos_s,
									 const double yaw_s,
									 const Eigen::Vector3d& pos_c,
									 const double yaw_c );
		bool check_waypoint_complete( const ros::Time t );
		void reset_waypoint_timer( void );

		//Calculates the radial distance between two points
		double radial_dist( const Eigen::Vector3d a, const Eigen::Vector3d b );
		double rotation_dist( const double a, const double b );

		//Convinence functions for generating a position target from a pose
		mavros_msgs::PositionTarget target_from_pose( const ros::Time& time,
													  const std::string& frame_id,
													  const geometry_msgs::Pose& p );
		mavros_msgs::PositionTarget target_from_pose( const ros::Time& time,
													  const std::string& frame_id,
													  const Eigen::Affine3d& g );

		double yaw_from_quaternion( const geometry_msgs::Quaternion &q );
		double yaw_from_quaternion( const Eigen::Quaterniond &q );
		Eigen::Quaterniond quaternion_from_yaw( const double yaw );

		//Spline helper functions
		//Get the interpolated point from the spline,
		//	t should be the normalized time
		/*
		inline void get_spline_reference(Eigen::Vector3d& p_interp, Eigen::Vector3d& v_interp, double& yaw, double& yaw_rate, const double t) const {
			// x values need to be scaled down in extraction as well.
			ROS_ASSERT_MSG((t >= 0.0) && (t <= 1.0), "Invalid time point given for spline interpolation (0.0 <= t <= 1.0)");

			//We only want the 0th and 1st-order derivatives
			const Eigen::Matrix<double,4,2> sp = spline_p_.derivatives(t,1);
			const Eigen::Matrix<double,2,2> sr = spline_r_.derivatives(t,1);
			p_interp = sp.block<3,1>(1,0);
			v_interp = sp.block<3,1>(1,1);
			yaw = sr(1,0);
			yaw_rate = sr(1,1);
		}
		*/
		inline void get_spline_reference(Eigen::Vector3d& p_interp,
										 Eigen::Vector3d& v_interp,
										 double& yaw,
										 double& yaw_rate,
										 const double u) const {
			// x values need to be scaled down in extraction as well.
			ROS_ASSERT_MSG((u >= 0.0) && (u <= 1.0), "Invalid time point given for spline interpolation (0.0 <= t <= 1.0)");


			std::vector<tinyspline::real> vx = spline_x_(u).result();
			std::vector<tinyspline::real> vy = spline_y_(u).result();
			std::vector<tinyspline::real> vz = spline_z_(u).result();
			std::vector<tinyspline::real> vyaw = spline_yaw_(u).result();

			p_interp = Eigen::Vector3d(vx[0],vy[0],vz[0]);
			yaw = vyaw[0];

			/*
			std::vector<tinyspline::real> vxd = spline_xd_(u).result();
			std::vector<tinyspline::real> vyd = spline_yd_(u).result();
			std::vector<tinyspline::real> vzd = spline_zd_(u).result();
			std::vector<tinyspline::real> vyawd = spline_yawd_(u).result();

			v_interp = Eigen::Vector3d(vxd[0],vyd[0],vzd[0]);
			yaw_rate = vyawd[0];
			*/

			//XXX: Manually derrive over a short period as proper derivative can't be calculated using this library
			double dt = 0.02;
			//Shorten time to ensure that 0.0<=u<=1.0 is preserved
			double ul = u - dt;
			double uh = u + dt;
			ul = (ul >= 0.0) ? ul : 0.0;
			uh = (uh <= 1.0) ? uh : 1.0;

			std::vector<tinyspline::real> vxdl = spline_x_(ul).result();
			std::vector<tinyspline::real> vydl = spline_y_(ul).result();
			std::vector<tinyspline::real> vzdl = spline_z_(ul).result();
			std::vector<tinyspline::real> vyawdl = spline_yaw_(ul).result();
			std::vector<tinyspline::real> vxdh = spline_x_(uh).result();
			std::vector<tinyspline::real> vydh = spline_y_(uh).result();
			std::vector<tinyspline::real> vzdh = spline_z_(uh).result();
			std::vector<tinyspline::real> vyawdh = spline_yaw_(uh).result();

			v_interp.x() = (vxdh[0] - vxdl[0]) / (2*dt);
			v_interp.y() = (vydh[0] - vydl[0]) / (2*dt);
			v_interp.z() = (vzdh[0] - vzdl[0]) / (2*dt);
			yaw_rate = (vyawdh[0] - vyawdl[0]) / (2*dt);
		}

		inline void get_dsp_spline_reference(Eigen::Vector3d& p_interp,
										 Eigen::Vector3d& v_interp,
										 double& yaw,
										 double& yaw_rate,
										 const double u) const {
			// x values need to be scaled down in extraction as well.
			ROS_ASSERT_MSG((u >= 0.0) && (u <= 1.0), "Invalid time point given for spline interpolation (0.0 <= t <= 1.0)");


			std::vector<tinyspline::real> vx = dsp_spline_x_(u).result();
			std::vector<tinyspline::real> vy = dsp_spline_y_(u).result();
			std::vector<tinyspline::real> vz = dsp_spline_z_(u).result();
			std::vector<tinyspline::real> vyaw = dsp_spline_yaw_(u).result();

			p_interp = Eigen::Vector3d(vx[0],vy[0],vz[0]);
			yaw = vyaw[0];

			/*
			std::vector<tinyspline::real> vxd = spline_xd_(u).result();
			std::vector<tinyspline::real> vyd = spline_yd_(u).result();
			std::vector<tinyspline::real> vzd = spline_zd_(u).result();
			std::vector<tinyspline::real> vyawd = spline_yawd_(u).result();

			v_interp = Eigen::Vector3d(vxd[0],vyd[0],vzd[0]);
			yaw_rate = vyawd[0];
			*/

			//XXX: Manually derrive over a short period as proper derivative can't be calculated using this library
			double dt = 0.02;
			//Shorten time to ensure that 0.0<=u<=1.0 is preserved
			double ul = u - dt;
			double uh = u + dt;
			ul = (ul >= 0.0) ? ul : 0.0;
			uh = (uh <= 1.0) ? uh : 1.0;

			std::vector<tinyspline::real> vxdl = dsp_spline_x_(ul).result();
			std::vector<tinyspline::real> vydl = dsp_spline_y_(ul).result();
			std::vector<tinyspline::real> vzdl = dsp_spline_z_(ul).result();
			std::vector<tinyspline::real> vyawdl = dsp_spline_yaw_(ul).result();
			std::vector<tinyspline::real> vxdh = dsp_spline_x_(uh).result();
			std::vector<tinyspline::real> vydh = dsp_spline_y_(uh).result();
			std::vector<tinyspline::real> vzdh = dsp_spline_z_(uh).result();
			std::vector<tinyspline::real> vyawdh = dsp_spline_yaw_(uh).result();

			v_interp.x() = (vxdh[0] - vxdl[0]) / (2*dt);
			v_interp.y() = (vydh[0] - vydl[0]) / (2*dt);
			v_interp.z() = (vzdh[0] - vzdl[0]) / (2*dt);
			yaw_rate = (vyawdh[0] - vyawdl[0]) / (2*dt);
		}

		// Helpers to scale X values down to [0, 1]
		inline double normalize(double x, const double min, const double max) const {
			return (x - min) / (max - min);
		}

		inline void make_yaw_continuous( std::vector<double>& yaw ) {
			for(int i=1; i<yaw.size(); i++) {
				while(fabs(yaw[i] - yaw[i-1]) > M_PI) {
					if(yaw[i] > yaw[i-1]) {
						yaw[i] -= 2*M_PI;
					} else {
						yaw[i] += 2*M_PI;
					}
				}
			}
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
