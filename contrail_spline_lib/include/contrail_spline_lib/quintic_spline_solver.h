#ifndef CONTRAIL_SPLINE_LIB_QUINTIC_SPLINE_SOLVER_H
#define CONTRAIL_SPLINE_LIB_QUINTIC_SPLINE_SOLVER_H

#include <contrail_spline_lib/quintic_spline_types.h>

#include <eigen3/Eigen/Dense>

namespace contrail_spline_lib {

class QuinticSplineSolver {
	public:
		void add( std::vector<double> vec );

		quintic_spline_coeffs_t solver( const double q0,
										const double qd0,
										const double qdd0,
										const double qf,
										const double qdf,
										const double qddf );

		Eigen::VectorXd linear_derivative_est( const Eigen::VectorXd& vias,
											   const double dt );

		quintic_spline_point_t lookup(const double u, const quintic_spline_coeffs_t& c);
};

}

#endif
