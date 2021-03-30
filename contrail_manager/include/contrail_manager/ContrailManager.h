#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <contrail_manager/TrajectoryAction.h>
#include <contrail_manager/ManagerParamsConfig.h>
#include <contrail_spline_lib/interpolated_quintic_spline.h>

#include <actionlib/server/simple_action_server.h>

#include <mavros_msgs/PositionTarget.h>

#include <eigen3/Eigen/Dense>

#include <vector>
#include <string>
#include <math.h>

class ContrailManager {
	private:
		ros::NodeHandle nhp_;

		ros::Publisher pub_is_ready_;		//Publishes feedback from the parent node to show when we will accept inputs
		ros::Publisher pub_spline_approx_;	//Publishes a approximate visualization of the calculated spline as feedback
		ros::Publisher pub_spline_points_;	//Publishes a path representing the interpolated spline points

		dynamic_reconfigure::Server<contrail_manager::ManagerParamsConfig> dyncfg_settings_;

		std::string param_frame_id_;
		int param_spline_approx_res_;
		double param_end_position_accuracy_;
		double param_end_yaw_accuracy_;
		bool param_ref_position_;
		bool param_ref_velocity_;
		bool param_ref_acceleration_;

		ros::Time spline_start_;
		ros::Duration spline_duration_;
		bool spline_in_progress_;
		bool is_ready_;
		bool wait_reached_end_;
		Eigen::Vector3d spline_pos_start_;
		Eigen::Vector3d spline_pos_end_;
		double spline_rot_start_;
		double spline_rot_end_;

		contrail_spline_lib::InterpolatedQuinticSpline spline_x_;
		contrail_spline_lib::InterpolatedQuinticSpline spline_y_;
		contrail_spline_lib::InterpolatedQuinticSpline spline_z_;
		contrail_spline_lib::InterpolatedQuinticSpline spline_r_;

		Eigen::Vector3d output_pos_last_;
		double output_rot_last_;

		actionlib::SimpleActionServer<contrail_manager::TrajectoryAction> as_;

	public:
		ContrailManager( const ros::NodeHandle &nh, std::string frame_id = "map", const bool is_ready = false );

		~ContrailManager( void );

		void set_frame_id( std::string frame_id );

		bool has_reference( const ros::Time t );
		void clear_reference( void );

		//Parent node/library should must indicate to contrail that it is ready to go
		//before contrail will accept new goals. Setting to false will cause contrail
		//to reject new goals (won't affect current goal, parent must use clear_reference())
		void allow_new_goals( const bool is_active );
		bool is_allowing_new_goals( void );

		//Gets the current reference from the latest updated source
		//Returns true if the reference was successfully obtained
		//Also performs checks on whether end has reached succsesfully
		bool get_reference( mavros_msgs::PositionTarget &ref,
							const ros::Time tc,
							const geometry_msgs::Pose &pose );

		bool get_reference( mavros_msgs::PositionTarget &ref,
							const ros::Time tc,
							const Eigen::Affine3d &g_c );

		bool get_reference( Eigen::Vector3d &pos,
							Eigen::Vector3d &vel,
							Eigen::Vector3d &acc,
							double &rpos,
							double &rrate,
							const ros::Time tc,
							const Eigen::Affine3d &g_c );

		void check_end_reached( const geometry_msgs::Pose &p_c );
		void check_end_reached( const Eigen::Affine3d &g_c );

	private:
		//ROS callbacks
		void callback_cfg_settings( contrail_manager::ManagerParamsConfig &config, uint32_t level );
		void callback_actionlib_goal(void);
		void callback_actionlib_preempt(void);

		void set_action_goal();

		void get_spline_reference(contrail_spline_lib::InterpolatedQuinticSpline& spline, double& pos, double& vel, double& acc, const double u);

		inline double normalize(double x, const double min, const double max) const {
			return (x - min) / (max - min);
		}

		std::vector<double> make_yaw_continuous( const std::vector<double>& yaw );

		void publish_approx_spline( const ros::Time& stamp );
		void publish_spline_points( const ros::Time& stamp, const std::vector<geometry_msgs::Vector3>& pos, const std::vector<double>& yaw );

		//Returns true of the tracking point has been reached
		bool check_endpoint_reached( const Eigen::Vector3d& pos_s,
									 const double yaw_s,
									 const Eigen::Vector3d& pos_c,
									 const double yaw_c );

		//Calculates the radial distance between two points
		double radial_dist( const Eigen::Vector3d& a, const Eigen::Vector3d& b );
		double yaw_error_shortest_path(const double y_sp, const double y);
		//double rotation_dist( const double a, const double b );

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
