#include <boost/python.hpp>

#include <contrail_spline_lib/quintic_spline_solver.h>

#include <vector>

class QuinticSplineSolverWrapper : public contrail_spline_lib::QuinticSplineSolver {
	public:
		QuinticSplineSolverWrapper() : QuinticSplineSolver() {}

		void _add(boost::python::list& list) {
			std::vector<double> vec;

			for (int i = 0; i < len(list); ++i)
				vec.push_back( boost::python::extract<double>(list[i]) );

			add(vec);
		}
};

BOOST_PYTHON_MODULE(_quintic_spline_solver_wrapper_cpp) {
	boost::python::class_<QuinticSplineSolverWrapper>
		( "QuinticSplineSolverWrapper", boost::python::init<>() )
		.def("add", &QuinticSplineSolverWrapper::_add)
		;
}
