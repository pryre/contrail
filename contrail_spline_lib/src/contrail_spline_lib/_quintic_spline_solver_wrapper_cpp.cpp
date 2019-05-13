#include <boost/python.hpp>

#include <contrail_spline_lib/quintic_spline_types.h>
#include <contrail_spline_lib/quintic_spline_solver.h>

#include <vector>

class QuinticSplineSolverWrapper : public contrail_spline_lib::QuinticSplineSolver {
	public:
		QuinticSplineSolverWrapper() : QuinticSplineSolver() {}

		boost::python::list _solver( const double q0,
									 const double qd0,
									 const double qdd0,
									 const double qf,
									 const double qdf,
									 const double qddf ) {

			contrail_spline_lib::quintic_spline_coeffs_t c = solver(q0, qd0, qdd0, qf, qdf,qddf);

			boost::python::list list;
			list.append<double>(c.a1);
			list.append<double>(c.a2);
			list.append<double>(c.a3);
			list.append<double>(c.a4);
			list.append<double>(c.a5);
			list.append<double>(c.a6);

			return list;
		}

		boost::python::list _linear_derivative_est( const boost::python::list& vias_list,
												   const double dt ) {

			Eigen::VectorXd vias = Eigen::VectorXd::Zero(len(vias_list));
			for (size_t i = 0; i < len(vias_list); ++i)
				vias(i) = boost::python::extract<double>(vias_list[i]);

			Eigen::VectorXd dvias = linear_derivative_est(vias, dt);

			boost::python::list list;
			for (size_t i = 0; i < dvias.size(); ++i)
				list.append<double>( dvias(i) );

			return list;
		}

		boost::python::list _lookup(const double u, const boost::python::list& coeffs) {
			boost::python::list list;

			if (len(coeffs) == 6) {
				contrail_spline_lib::quintic_spline_coeffs_t c;
				c.a1 = boost::python::extract<double>(coeffs[0]);
				c.a2 = boost::python::extract<double>(coeffs[1]);
				c.a3 = boost::python::extract<double>(coeffs[2]);
				c.a4 = boost::python::extract<double>(coeffs[3]);
				c.a5 = boost::python::extract<double>(coeffs[4]);
				c.a6 = boost::python::extract<double>(coeffs[5]);
				contrail_spline_lib::quintic_spline_point_t point = lookup(u, c);

				list.append<double>(point.q);
				list.append<double>(point.qd);
				list.append<double>(point.qdd);
			}

			return list;
		}
};

BOOST_PYTHON_MODULE(_quintic_spline_solver_wrapper_cpp) {
	boost::python::class_<QuinticSplineSolverWrapper>
		( "QuinticSplineSolverWrapper", boost::python::init<>() )
		.def("solver", &QuinticSplineSolverWrapper::_solver)
		.def("linear_derivative_est", &QuinticSplineSolverWrapper::_linear_derivative_est)
		.def("lookup", &QuinticSplineSolverWrapper::_lookup)
		;
}
