#ifndef CONTRAIL_SPLINE_LIB_INTERPOLATE_QUINTIC_SPLINE_H
#define CONTRAIL_SPLINE_LIB_INTERPOLATE_QUINTIC_SPLINE_H

#include <contrail_spline_lib/quintic_spline_types.h>
#include <contrail_spline_lib/quintic_spline_solver.h>

#include <eigen3/Eigen/Dense>

#include <vector>

namespace contrail_spline_lib {

class InterpolatedQuinticSpline {
	private:
		std::vector<quintic_spline_coeffs_t> _subsplines;

		Eigen::VectorXd _vias;
		Eigen::VectorXd _dvias;
		Eigen::VectorXd _ddvias;

		QuinticSplineSolver _solver;

		bool _is_valid;

	public:
		InterpolatedQuinticSpline( void );
		~InterpolatedQuinticSpline( void );

		bool interpolate( const Eigen::VectorXd& vias );

		const Eigen::VectorXd& get_vias( void );
		const Eigen::VectorXd& get_dvias( void );
		const Eigen::VectorXd& get_ddvias( void );

		quintic_spline_point_t lookup( double u );

		const inline bool is_valid( void ) { return _is_valid; };
};

}

#endif
