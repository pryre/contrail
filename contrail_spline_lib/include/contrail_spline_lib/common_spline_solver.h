#ifndef CONTRAIL_SPLINE_LIB_COMMON_SPLINE_SOLVER_H
#define CONTRAIL_SPLINE_LIB_COMMON_SPLINE_SOLVER_H

#include <contrail_spline_lib/common_spline_types.h>

#include <eigen3/Eigen/Dense>

namespace contrail_spline_lib {

const unsigned int dim_sys_cubic = 4;
const unsigned int dim_sys_quintic = 6;
const unsigned int dim_sys_septic = 8;
const unsigned int dim_sys_nonic = 10;

template<std::size_t N>
std::array<double,N> polyder(const std::array<double,N>& c);

template<std::size_t N>
constexpr Eigen::Matrix<double,2*N,2*N> spline_solver_gen_tnorm_ls();

template<std::size_t N>
constexpr Eigen::Matrix<double,2*N,2*N> normalised_spline_solver_gen_inv_sys();

template<std::size_t N>
void normalised_spline_solver(normalised_spline_t<N>& spline);

template<std::size_t N>
spline_via_t<N> normalised_spline_lookup(const polynomial_segment_t<N>& seg, const double ndt);

/*
const

class CommonSplineSolver {
	public:
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
*/

}

#endif
