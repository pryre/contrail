#ifndef CONTRAIL_SPLINE_LIB_QUINTIC_SPLINE_TYPES_H
#define CONTRAIL_SPLINE_LIB_QUINTIC_SPLINE_TYPES_H

#include <vector>

namespace contrail_spline_lib {

typedef struct {
	double a1;
	double a2;
	double a3;
	double a4;
	double a5;
	double a6;
} quintic_spline_coeffs_t;

typedef struct {
	double q;
	double qd;
	double qdd;
} quintic_spline_point_t;

typedef struct {
	std::vector<quintic_spline_coeffs_t> seg_coeffs;
	double duration;
} multi_segment_quintic_spline_t;

}

#endif
