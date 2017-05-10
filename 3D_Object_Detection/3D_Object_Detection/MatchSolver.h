//该类封装了 dlib 提供的非线性最优化方法
#pragma once
#ifndef MATCH_SOLVER
#define MATCH_SOLVER

#include <MatchEdges.h>
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <dlib\optimization.h>

class MatchSolver: public MatchEdges {
public:
	typedef dlib::matrix<double, 1, 1> input_vector;
	typedef dlib::matrix<double, 6, 1> parameter_vector;
	typedef dlib::matrix<double, 1, 1> output_vector;
	parameter_vector var;
	void setIniVar(double x, double y, double z, double x_deg, double y_deg, double z_deg) {
		var(0) = x;
		var(1) = y;
		var(2) = z;
		var(3) = x_deg;
		var(4) = y_deg;
		var(5) = z_deg;

		data_samples.push_back(make_pair(input_vector(0), output_vector(0)));
	};
	MatchSolver(Mat * cam_img_input) :MatchEdges(cam_img_input) {};

	const double residual (
		const std::pair<input_vector, double>& data,
		const parameter_vector& params
	)const
	{
		double params_array[6];
		params_array[0] = params(0);
		params_array[1] = params(1);
		params_array[2] = params(2);
		params_array[3] = params(3);
		params_array[4] = params(4);
		params_array[5] = params(5);
		return hausdorffDistance(params_array);
	}

	void solve(Mat *cam_img_input, double* final_result) {
		
		const double (MatchSolver::*residual_ptr)(
			const std::pair<input_vector, double>& data,
			const parameter_vector& params
			) const = NULL;
		residual_ptr = &MatchSolver::residual;
		
		dlib::solve_least_squares_lm(dlib::objective_delta_stop_strategy(1e-7).be_verbose(),
			MatchSolver::residual,
			dlib::derivative(MatchSolver::residual),
			data_samples,
			var,1);
		

		std::cout << "The final position and rotation are:  x:" << var(0) << " y: " << var(1) << " z: " << var(2) << " x_deg:" << var(3) << " y_deg:" << var(4) << " z_deg:" << var(5) << std::endl;
		
		final_result[0] = var(0);
		final_result[1] = var(1);
		final_result[2] = var(2);
		final_result[3] = var(3);
		final_result[4] = var(4);
		final_result[5] = var(5);
	};
	

private:
	std::vector<std::pair<input_vector, output_vector> > data_samples;
};




#endif
