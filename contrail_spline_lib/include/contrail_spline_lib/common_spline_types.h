#ifndef CONTRAIL_SPLINE_LIB_COMMON_SPLINE_TYPES_H
#define CONTRAIL_SPLINE_LIB_COMMON_SPLINE_TYPES_H

#include <vector>
#include <array>

namespace contrail_spline_lib {

template <std::size_t N>
using spline_via_t = std::array<double,N>;

template <std::size_t N>
using polynomial_coeffs_t = std::array<std::vector<double>,N>;


template<std::size_t N>
struct polynomial_segment_t {
	spline_via_t<N> start;
	spline_via_t<N> end;
	polynomial_coeffs_t<N> coeffs;
	double duration;

	const size_t order = 2*N-1;
};

template<std::size_t N>
using normalised_spline_t = std::vector<polynomial_segment_t<N>>;

}

#endif
