#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <contrail/ManagerParamsConfig.h>

#include <tinyspline_ros/tinysplinecpp.h>
#include <actionlib/server/simple_action_server.h>
#include <contrail/TrajectoryAction.h>

#include <mavros_msgs/PositionTarget.h>

#include <eigen3/Eigen/Dense>

#include <vector>
#include <string>
#include <math.h>

class ContrailManager {
	private:
		ros::NodeHandle& nhp_;

		ros::Publisher pub_spline_approx_;	//Publishes a approximate visualization of the calculated spline as feedback
		ros::Publisher pub_spline_points_;	//Publishes a path representing the interpolated spline points

		dynamic_reconfigure::Server<contrail::ManagerParamsConfig> dyncfg_settings_;

		std::string param_frame_id_;
		int param_spline_approx_res_;
		double param_end_position_accuracy_;
		double param_end_yaw_accuracy_;

		ros::Time spline_start_;
		ros::Duration spline_duration_;
		bool spline_in_progress_;
		bool wait_reached_end_;
		Eigen::Vector3d spline_pos_start_;
		Eigen::Vector3d spline_pos_end_;
		double spline_rot_start_;
		double spline_rot_end_;
		tinyspline::BSpline spline_x_;
		tinyspline::BSpline spline_xd_;
		tinyspline::BSpline spline_y_;
		tinyspline::BSpline spline_yd_;
		tinyspline::BSpline spline_z_;
		tinyspline::BSpline spline_zd_;
		tinyspline::BSpline spline_r_;
		tinyspline::BSpline spline_rd_;
		bool use_dirty_derivative_;

		Eigen::Vector3d output_pos_last_;
		double output_rot_last_;

		ros::Timer timer_;
		actionlib::SimpleActionServer<contrail::TrajectoryAction> as_;

	public:
		ContrailManager( ros::NodeHandle nh, std::string frame_id = "map" );

		~ContrailManager( void );

		bool has_reference( const ros::Time t );

		//Gets the current reference from the latest updated source
		//Returns true if the reference was successfully obtained
		bool get_reference( mavros_msgs::PositionTarget &ref,
							const ros::Time tc,
							const geometry_msgs::Pose &pose );

		bool get_reference( mavros_msgs::PositionTarget &ref,
							const ros::Time tc,
							const Eigen::Affine3d &g_c );

		bool get_reference( Eigen::Vector3d &pos,
							Eigen::Vector3d &vel,
							double &rpos,
							double &rrate,
							const ros::Time tc );

		void check_end_reached( const geometry_msgs::Pose &p_c );
		void check_end_reached( const Eigen::Affine3d &g_c );

	private:
		//ROS callbacks
		void callback_cfg_settings( contrail::ManagerParamsConfig &config, uint32_t level );
		void callback_timer(const ros::TimerEvent& e);

		void set_action_goal();

		void get_spline_reference(tinyspline::BSpline& spline, tinyspline::BSpline& splined, double& pos, double& vel, const double u);

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

		void publish_approx_spline( const ros::Time& stamp );
		void publish_spline_points( const ros::Time& stamp, const std::vector<geometry_msgs::Vector3>& pos, const std::vector<double>& yaw );

		//Returns true of the tracking point has been reached
		bool check_endpoint_reached( const Eigen::Vector3d& pos_s,
									 const double yaw_s,
									 const Eigen::Vector3d& pos_c,
									 const double yaw_c );

		//Calculates the radial distance between two points
		double radial_dist( const Eigen::Vector3d& a, const Eigen::Vector3d& b );
		double rotation_dist( const double a, const double b );

		double yaw_from_quaternion( const Eigen::Quaterniond &q );
		Eigen::Quaterniond quaternion_from_yaw( const double yaw );

		Eigen::Vector3d position_from_msg( const geometry_msgs::Point &p );
		Eigen::Quaterniond quaternion_from_msg( const geometry_msgs::Quaternion &q );
		Eigen::Vector3d vector_from_msg( const geometry_msgs::Vector3 &v );
		Eigen::Affine3d affine_from_msg( const geometry_msgs::Pose &pose );

		geometry_msgs::Point point_from_eig( const Eigen::Vector3d &p );
		geometry_msgs::Quaternion quaternion_from_eig( const Eigen::Quaterniond &q );
		geometry_msgs::Vector3 vector_from_eig( const Eigen::Vector3d &v );
		geometry_msgs::Pose pose_from_eig( const Eigen::Affine3d &g );
};
