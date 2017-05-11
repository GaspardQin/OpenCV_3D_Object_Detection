//该类封装了 dlib 提供的非线性最优化方法
#pragma once
#ifndef MATCH_SOLVER
#define MATCH_SOLVER

#include <MatchEdges.h>
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <dlib\optimization.h>
using namespace dlib;


class MatchSolver{
public:
	typedef matrix<double, 2, 1> input_vector;
	typedef matrix<double, 6, 1> parameter_vector;
	typedef matrix<double, 1, 1> output_vector;
	//typedef dlib::matrix<double, 6, 1> column_vector;
	//column_vector var;
	parameter_vector var;
	std::vector<std::pair<input_vector, double> > data_samples;
	void setIniVar(double x, double y, double z, double x_deg, double y_deg, double z_deg) {
		var(0) = x;
		var(1) = y;
		var(2) = z;
		var(3) = x_deg;
		var(4) = y_deg;
		var(5) = z_deg;
		input_vector input_vector_false;
		input_vector_false(0) = 1;
		input_vector_false(1) = 1;
		data_samples.push_back(pair<input_vector, double>(input_vector_false, 0));

	};
	MatchSolver(Mat * cam_img_input) { cam_img = cam_img_input; };
	/*
	class  CostFactor :public MatchEdges {
	public:
		typedef MatchSolver::column_vector column_vector;
		typedef dlib::matrix<double> general_matrix;
		Mat *cam_img;

		CostFactor(Mat *cam_img_input) : MatchEdges(cam_img_input) {
			cam_img = cam_img_input;
		}
		double operator() (
			const column_vector& params
			) const {
			double params_array[6];
			params_array[0] = params(0);
			params_array[1] = params(1);
			params_array[2] = params(2);
			params_array[3] = params(3);
			params_array[4] = params(4);
			params_array[5] = params(5);
			return hausdorffDistance(params_array);
		}


		void get_derivative_and_hessian(
			const column_vector& params,
			column_vector& der,
			general_matrix& hess
		) const
		{
			double params_array[6];
			params_array[0] = params(0);
			params_array[1] = params(1);
			params_array[2] = params(2);
			params_array[3] = params(3);
			params_array[4] = params(4);
			params_array[5] = params(5);
			//der = dlib::derivative(CostFactor(cam_img))(params_array);
			//hess = rosen_hessian(x);
		}
	};
	class CostDerivative :public CostFactor {
	public:

		double d_step;
		CostDerivative(double d_step_input, Mat *cam_img_input):CostFactor(cam_img_input) {
			d_step = d_step_input;
		}
		const column_vector operator() (
			const column_vector& params
			) const {
			column_vector der;//梯度向量
			column_vector small_var;//微小变量
			
			
			small_var = d_step, 0, 0, 0, 0, 0;
			der(0) = partDerCentre(params, small_var);

			small_var = 0, d_step, 0, 0, 0, 0;
			der(1) = partDerCentre(params, small_var);

			small_var = 0, 0, d_step, 0, 0, 0;
			der(2) = partDerCentre(params, small_var);

			small_var = 0, 0, 0, d_step, 0, 0;
			der(3) = partDerCentre(params, small_var);

			small_var = 0, 0, 0, 0, d_step, 0;
			der(4) = partDerCentre(params, small_var);

			small_var = 0, 0, 0, 0, 0, d_step;
			der(5) = partDerCentre(params, small_var);
			return der;
		}
		double partDerCentre (const column_vector& params, const column_vector& d_params)const {
			//利用centre法则求解偏导，d_params代表移动的某方向正小量，例如(0,0,0,0,0,0.01)
			double params_array_l[6], params_array_r[6];
			
			params_array_l[0] = params(0) - d_params(0);
			params_array_l[1] = params(1) - d_params(1);
			params_array_l[2] = params(2) - d_params(2);
			params_array_l[3] = params(3) - d_params(3);
			params_array_l[4] = params(4) - d_params(4);
			params_array_l[5] = params(5) - d_params(5);

			params_array_r[0] = params(0) + d_params(0);
			params_array_r[1] = params(1) + d_params(1);
			params_array_r[2] = params(2) + d_params(2);
			params_array_r[3] = params(3) + d_params(3);
			params_array_r[4] = params(4) + d_params(4);
			params_array_r[5] = params(5) + d_params(5);

			return ((hausdorffDistance(params_array_r) - hausdorffDistance(params_array_l)) / length(d_params));
		}
	};


	void solve(double* final_result) {


		find_min(bfgs_search_strategy(),  // Use BFGS search algorithm
			objective_delta_stop_strategy(1e-3), // Stop when the change in rosen() is less than 1e-7
			CostFactor(cam_img), CostDerivative(0.001,cam_img), var, -1);
		// Once the function ends the starting_point vector will contain the optimum point 
		// of (1,1).


		cout << "test_function solution:\n" << var << endl;
		

		std::cout << "The final position and rotation are:  x:" << var(0) << " y: " << var(1) << " z: " << var(2) << " x_deg:" << var(3) << " y_deg:" << var(4) << " z_deg:" << var(5) << std::endl;
		
		final_result[0] = var(0);
		final_result[1] = var(1);
		final_result[2] = var(2);
		final_result[3] = var(3);
		final_result[4] = var(4);
		final_result[5] = var(5);
	};
	*/


class residual :public MatchEdges {
public:
	double operator() (const std::pair<input_vector, double>& data,
		const parameter_vector& params) const {
		double params_array[6];
		params_array[0] = params(0);
		params_array[1] = params(1);
		params_array[2] = params(2);
		params_array[3] = params(3);
		params_array[4] = params(4);
		params_array[5] = params(5);
		double dist = hausdorffDistance(params_array);
		cout << "Hausdorff_distance " << dist << endl;
		return dist;
	}
	residual(Mat* cam_img_input) :MatchEdges(cam_img_input) {};

};
class CostDerivative :public residual {
public:

	double d_step;
	CostDerivative(double d_step_input, Mat *cam_img_input) :residual(cam_img_input) {
		d_step = d_step_input;
	}
	const parameter_vector operator() (
		const std::pair<input_vector, double>& data,
		const parameter_vector& params
		) const {
		parameter_vector der;//梯度向量
		parameter_vector small_var;//微小变量


		small_var = d_step, 0, 0, 0, 0, 0;
		der(0) = partDerCentre(params, small_var);

		small_var = 0, d_step, 0, 0, 0, 0;
		der(1) = partDerCentre(params, small_var);

		small_var = 0, 0, d_step, 0, 0, 0;
		der(2) = partDerCentre(params, small_var);

		small_var = 0, 0, 0, d_step/10, 0, 0;
		der(3) = partDerCentre(params, small_var);

		small_var = 0, 0, 0, 0, d_step/10, 0;
		der(4) = partDerCentre(params, small_var);

		small_var = 0, 0, 0, 0, 0, d_step/10;
		der(5) = partDerCentre(params, small_var);
		
		return der;
	}
	double partDerCentre(const parameter_vector& params, const parameter_vector& d_params)const {
		//利用centre法则求解偏导，d_params代表移动的某方向正小量，例如(0,0,0,0,0,0.01)
		double params_array_l[6], params_array_r[6];

		params_array_l[0] = params(0) - d_params(0);
		params_array_l[1] = params(1) - d_params(1);
		params_array_l[2] = params(2) - d_params(2);
		params_array_l[3] = params(3) - d_params(3);
		params_array_l[4] = params(4) - d_params(4);
		params_array_l[5] = params(5) - d_params(5);

		params_array_r[0] = params(0) + d_params(0);
		params_array_r[1] = params(1) + d_params(1);
		params_array_r[2] = params(2) + d_params(2);
		params_array_r[3] = params(3) + d_params(3);
		params_array_r[4] = params(4) + d_params(4);
		params_array_r[5] = params(5) + d_params(5);
		double d = (hausdorffDistance(params_array_r) - hausdorffDistance(params_array_l)) / length(d_params) / 2;
		cout << "'der " << d << endl;
		return (d);
	}
};
 void solve(double* final_result) {
	 parameter_vector var_ = var;
	 dlib::solve_least_squares(objective_delta_stop_strategy(1e-1).be_verbose(),
		 residual(cam_img),
		 CostDerivative(0.5,cam_img),
		 data_samples,
		 var_);
	 cout << "test_function solution:\n" << var_ << endl;


	 std::cout << "The final position and rotation are:  x:" << var_(0) << " y: " << var_(1) << " z: " << var_(2) << " x_deg:" << var_(3) << " y_deg:" << var_(4) << " z_deg:" << var_(5) << std::endl;

	 final_result[0] = var(0);
	 final_result[1] = var(1);
	 final_result[2] = var(2);
	 final_result[3] = var(3);
	 final_result[4] = var(4);
	 final_result[5] = var(5);
 }
private:
	
	cv::Mat * cam_img;
};




#endif
