#include <contrail_spline_lib/common_spline_solver.h>

#include <eigen3/Eigen/Dense>

#include <math.h>
#include <assert.h>

using namespace contrail_spline_lib;


//Returns the a vector representing the derivative of a polynomial
std::vector<double> polyder(const std::vector<double>& c) {
	std::vector<double> dc;
	if (c.size() > 1) {
		dc.reserve(c.size()-1);

		unsigned int n = c.size()-1;
		for(unsigned int i = 0; i < c.size()-1; i++) {
			dc.push_back(n*c[i]);
			n--;
		}
	}

	return dc;
}

// Generates a linear system of a set dimension for the use in solving for
// a spline in the case that the spline is time-normalised.
//	Inputs:
//		dim: Should be the dimension of the linear system, typically should
//			 be spline order + 1
//	Outputs:
//		M: The generated time-normalised linear system
template<std::size_t N>
constexpr Eigen::Matrix<double,2*N,2*N> contrail_spline_lib::spline_solver_gen_tnorm_ls() {
	Eigen::Matrix<double,2*N,2*N> M = Eigen::Matrix<double,2*N,2*N>::Zero();

	//const unsigned int dim = M.rows();
	//assert((dim > 0) && !(dim % 2));
	//assert((M.rows() == dim) && (M.cols() == dim));

    // Go through all columns
    for(unsigned int i=0; i < M.cols(); i++) {
        unsigned int order = i-1;
        unsigned int coeff = 1;

        // Go through both sets of t0 and tf simultaneously
		for(unsigned int j=0; j < (M.rows()/2u); j++) {
            if(order >= 0) {
                // Fill in t0 section
                M(j,i) = coeff*(pow(0,order));
                // Fill in tf section
                M((j+(M.rows()/2u)),i) = coeff*(pow(1,order));

                // "Derivation" step
                coeff = coeff*order;
                order = order - 1;
            } // Else M(j,i) == 0, but already initialised to 0
        }
    }
}

template<std::size_t N>
constexpr Eigen::Matrix<double,2*N,2*N> contrail_spline_lib::normalised_spline_solver_gen_inv_sys() {
	return spline_solver_gen_tnorm_ls<N>().inverse();
}

template<std::size_t N>
void contrail_spline_lib::normalised_spline_solver(normalised_spline_t<N>& spline) {
	//Calculate inverse polynomial system
	//M = spline_solver_gen_tnorm_ls( length(b) );
	//inv(M)
	Eigen::Matrix<double,2*N,2*N> Mi = normalised_spline_solver_gen_inv_sys<N>();

	for (auto& it : spline) {
		//b=[q0; qd0*dt; qdd0*(dt^2); qddd0*(dt^3); qdddd0*(dt^4); qf; qdf*dt; qddf*(dt^2); qdddf*(dt^3); qddddf*(dt^4)];
		Eigen::Matrix<double,N,1> b0(it.start.data());
		Eigen::Matrix<double,N,1> bf(it.end.data());

		//Normalise constraints (skip first as dt^0=1)
		for(unsigned int i = 1; i < b0.rows(); i++) {
			double nsdt = pow(it.duration,i);
			b0(i) = b0(i) * nsdt;
			bf(i) = bf(i) * nsdt;
		}

		//Fill out constraint vector
		Eigen::Matrix<double,2*N,1> b;
		b << b0, bf;

		//Calculate coefficients
		//a = inv(M)*b;
		//Reverse allocate as we want to get the coefficients in decending order of powers
		Eigen::Matrix<double,2*N,1>::Map(it.coeffs[0].data()) = (Mi*b).reverse();

		//...and coefficient derivatives
		for(unsigned int i = 1; i < it.coeffs.size(); i++)
			it.coeffs[i] = polyder(it.coeffs[i-1]);
	}
}


//Returns the denormalised via values
template<std::size_t N>
spline_via_t<N> normalised_spline_lookup(const polynomial_segment_t<N>& seg, const double ndt) {
	assert((ndt >= 0.0) && (ndt <= 1.0));

	/*
	spline_via_t<N> nq;
	std::fill(std::begin(nq), std::end(nq), 0.0);

	for(unsigned int i = 0; i < nq.size(); i++) {
		unsigned int n = c.size()-1;
		for(unsigned int j = 0; j < seg.coeffs[i].size()-1; j++) {
			nq[i] += seg.coeffs[i][j]*pow(ndt,n);
			n--;
		}
	}

	//Devide derivatives by segment dt to denormalise
	spline_via_t<N> q;

	q[0] = nq[0];
	for(unsigned int i = 1; i < q.size(); i++)
		q[i] = nq[i] / pow(seg.duration,i);

	*/

	spline_via_t<N> q;
	std::fill(std::begin(q), std::end(q), 0.0);

	// Loop through each level of derivatives
	for(unsigned int i = 0; i < q.size(); i++) {
		double nq = 0;

		unsigned int n = seg.coeffs[i].size()-1;
		for(unsigned int j = 0; j < seg.coeffs[i].size()-1; j++) {
			nq += seg.coeffs[i][j]*pow(ndt,n);
			n--;
		}

		if(i==0){
			q[i] = nq;
		} else {
			//Devide derivatives by segment dt to denormalise
			q[i] = nq / pow(seg.duration,i);
		}
	}

	return q;
}






