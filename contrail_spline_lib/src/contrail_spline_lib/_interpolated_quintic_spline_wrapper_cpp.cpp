#include <boost/python.hpp>

#include <contrail_spline_lib/interpolated_quintic_spline.h>
#include <contrail_spline_lib/quintic_spline_types.h>

#include <eigen3/Eigen/Dense>

class InterpolatedQuinticSplineWrapper : public contrail_spline_lib::InterpolatedQuinticSpline {
	public:
		InterpolatedQuinticSplineWrapper() : InterpolatedQuinticSpline() {}

		bool _interpolate( boost::python::list& list ) {
			Eigen::VectorXd vias = Eigen::VectorXd::Zero( len(list) );

			for (size_t i = 0; i < len(list); ++i)
				vias[i] = boost::python::extract<double>(list[i]);

			return interpolate(vias);
		}

		boost::python::list _get_vias( void ) {
			return _get_list_from_vec( get_vias() );
		}

		boost::python::list _get_dvias( void ) {
			return _get_list_from_vec( get_dvias() );
		}

		boost::python::list _get_ddvias( void ) {
			return _get_list_from_vec( get_ddvias() );
		}

		boost::python::list _lookup( double u ) {
			boost::python::list list;

			contrail_spline_lib::quintic_spline_point_t point = lookup(u);
			list.append<double>( point.q );
			list.append<double>( point.qd );
			list.append<double>( point.qdd );

			return list;
		}

	private:
		boost::python::list _get_list_from_vec(const Eigen::VectorXd& vec) {
			boost::python::list list;

			for (size_t i = 0; i < vec.size(); ++i)
				list.append<double>( vec(i) );

			return list;

		}
};

BOOST_PYTHON_MODULE(_interpolated_quintic_spline_wrapper_cpp) {
	boost::python::class_<InterpolatedQuinticSplineWrapper>
		( "InterpolatedQuinticSplineWrapper", boost::python::init<>() )
		.def("interpolate", &InterpolatedQuinticSplineWrapper::_interpolate)
		.def("get_vias", &InterpolatedQuinticSplineWrapper::_get_vias)
		.def("get_dvias", &InterpolatedQuinticSplineWrapper::_get_dvias)
		.def("get_ddvias", &InterpolatedQuinticSplineWrapper::_get_ddvias)
		.def("lookup", &InterpolatedQuinticSplineWrapper::_lookup)
		;
}
