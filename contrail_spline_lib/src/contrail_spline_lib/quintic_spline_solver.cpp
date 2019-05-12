#include <contrail_spline_lib/quintic_spline_solver.h>

#include <eigen3/Eigen/Dense>

#include <vector>

using namespace contrail_spline_lib;

void QuinticSplineSolver::add( std::vector<double> vec ) {
	for(size_t i = 0; i < vec.size(); i++) {
		printf("%i: %0.4f\n", i, vec[i]);
	}
}

Eigen::VectorXd linear__derivative_est( const Eigen::VectorXd& vias, const double dt )
function [dvias] = linear_est(vias, dt)
%LINEAR_EST Summary of this function goes here
%   Detailed explanation goes here

    dvias = zeros(length(vias),1);

    % If otherwise, it is a linear segment
    if length(vias) > 2
        for i = 2:length(vias)-1
            qp = vias(i-1,1);
            qc = vias(i,1);
            qn = vias(i+1,1);

%             dvias(i,1) = (qn - qp)/(2*dt);

            if (qc == qp) || (qc == qn) || ( (qc < qp) && (qc < qn) ) || ( (qc > qp) && (qc > qn) )
                dvias(i,1) = 0;
            else
                dvias(i,1) = (qn - qp)/(2*dt);
            end
        end
    end
end
