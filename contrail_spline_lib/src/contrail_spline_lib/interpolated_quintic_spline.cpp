#include <contrail_spline_lib/interpolated_quintic_spline.h>
#include <contrail_spline_lib/quintic_spline_solver.h>

using namespace contrail_spline_lib;

InterpolatedQuinticSpline::InterpolatedQuinticSpline( void ) :
	_is_valid(false) {

}

InterpolatedQuinticSpline::~InterpolatedQuinticSpline( void ) {
}


bool InterpolatedQuinticSpline::interpolate( const Eigen::VectorXd& vias ) {
	if( vias.size() >= 2 ) {
		_subsplines.clear();

		_vias = vias;
		_dvias = _solver.linear_derivative_est(_vias, 1.0);
		_ddvias = _solver.linear_derivative_est(_dvias, 1.0);

		for(size_t i=0; i < (_vias.size() - 1); i++) {
			_subsplines.push_back( _solver.solver( _vias(i),
												   _dvias(i),
												   _ddvias(i),
												   _vias(i+1),
												   _dvias(i+1),
												   _ddvias(i+1) ) );

		}

		_is_valid = true;
	}

	return _is_valid;
}

const Eigen::VectorXd& InterpolatedQuinticSpline::get_vias( void ) {
	return _vias;
}

const Eigen::VectorXd& InterpolatedQuinticSpline::get_dvias( void ) {
	return _dvias;
}

const Eigen::VectorXd& InterpolatedQuinticSpline::get_ddvias( void ) {

	return _ddvias;
}

quintic_spline_point_t InterpolatedQuinticSpline::lookup( double u ) {
	quintic_spline_point_t p;


	return p;
}
