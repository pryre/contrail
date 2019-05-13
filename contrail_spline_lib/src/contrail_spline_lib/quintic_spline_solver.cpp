#include <contrail_spline_lib/quintic_spline_solver.h>

#include <eigen3/Eigen/Dense>

#include <vector>
#include <stdio.h>
#include <math.h>

using namespace contrail_spline_lib;

void QuinticSplineSolver::add( std::vector<double> vec ) {
	for(size_t i = 0; i < vec.size(); i++) {
		printf("%i: %0.4f\n", (int)i, vec[i]);
	}
}


Eigen::VectorXd QuinticSplineSolver::linear_derivative_est( const Eigen::VectorXd& vias, const double dt ) {
	Eigen::VectorXd dvias = Eigen::VectorXd::Zero(vias.size());

	if( vias.size() > 2 ) {
		for(int i=2; i < (vias.size()-1); i++) {
			double qp = vias(i-1);
            double qc = vias(i);
            double qn = vias(i+1);

            if( (qc == qp) ||
				(qc == qn) ||
				( (qc < qp) && (qc < qn) ) ||
				( (qc > qp) && (qc > qn) ) ) {

                dvias(i) = 0;
            } else {
                dvias(i) = (qn - qp)/(2*dt);
            }
		}
	}
}

quintic_spline_coeffs_t QuinticSplineSolver::solver( const double q0,
													 const double qd0,
													 const double qdd0,
													 const double qf,
													 const double qdf,
													 const double qddf ) {

	// Computes the coefficients for a quintic polynomial reference trajectory
	//
	// Primary use of this solution is to provide continous acceleration (and
	// therefore a non-impulsive jerk), which should not induce vibrational
	// modes during operation.
	//
	// There are 6 constraints (3 initial, 3 final), therefore a 5th order
	// polynomial is required (this equation is derrived to get the equations
	// needed for the matrix representation):
	//		q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
	//
	// Calculations use normalized time input (t0=0; tf=1) for the calculation
	// to simplify the computations. This means that normalized time must be
	// used during lookup of the spline.
	//
	// Inputs:
	//	   q0: Initial position
	//	  qd0: Initial velocity
	//	 qdd0: Initial acceleration
	//	   qf: Final position
	//	  qdf: Final position
	//	 qddf: Final acceleration
	//
	// Simultaneous Polynomial Equations
	// M = [ 1, t0, t0^2,   t0^3,    t0^4,    t0^5;
	//       0,  1, 2*t0, 3*t0^2,  4*t0^3,  5*t0^4;
	//       0,  0,    2,   6*t0, 12*t0^2, 20*t0^3;
	//       1, tf, tf^2,   tf^3,    tf^4,    tf^5;
	//       0,  1, 2*tf, 3*tf^2,  4*tf^3,  5*tf^4;
	//       0,  0,    2,   6*tf, 12*tf^2, 20*tf^3];


	// Simultaneous Polynomial Equations
	Eigen::Matrix<double,6,6> M;
	M << 1, 0, 0, 0,  0,  0,
		 0, 1, 0, 0,  0,  0,
		 0, 0, 2, 0,  0,  0,
		 1, 1, 1, 1,  1,  1,
		 0, 1, 2, 3,  4,  5,
		 0, 0, 2, 6, 12, 20;

	// Coefficient Solver
    Eigen::Matrix<double,6,1> b;
	b << q0, qd0, qdd0, qf, qdf, qddf;
    Eigen::Matrix<double,6,1> a = M.fullPivLu().solve(b);

	quintic_spline_coeffs_t a_s;

	a_s.a1 = a(0);
	a_s.a2 = a(1);
	a_s.a3 = a(2);
	a_s.a4 = a(3);
	a_s.a5 = a(4);
	a_s.a6 = a(5);

	return a_s;
}

quintic_spline_point_t QuinticSplineSolver::lookup(const double u, const quintic_spline_coeffs_t& c) {
	quintic_spline_point_t p;

	p.q =     c.a1 +     c.a2*u +    c.a3*pow(u,2) +    c.a4*pow(u,3) +   c.a5*pow(u,4) + c.a6*pow(u,5);
	p.qd =    c.a2 +   2*c.a3*u +  3*c.a4*pow(u,2) +  4*c.a5*pow(u,3) + 5*c.a6*pow(u,4);
	p.qdd = 2*c.a3 +   6*c.a4*u + 12*c.a5*pow(u,2) + 20*c.a6*pow(u,3);
	//qddd =   6*a(4).*c +  24*a(5).*t + 60*a(6).*t.^2;
	//qdddd = 24*a(5).*c + 120*a(6).*t;

	return p;
}

