//该类封装了 Eigen 提供的非线性最优化方法
#pragma once
#ifndef MATCH_SOLVER
#define MATCH_SOLVER
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <Eigen/Core>
#include <MatchEdges.h>


class MatchSolver{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Eigen::VectorXf var = Eigen::VectorXf::Zero(6);
	

	void setIniVar(double x, double y, double z, double x_deg, double y_deg, double z_deg) {
		var(0) = x;
		var(1) = y;
		var(2) = z;
		var(3) = x_deg;
		var(4) = y_deg;
		var(5) = z_deg;
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

template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
class Functor
{
public:
	typedef _Scalar Scalar;
	enum {
		InputsAtCompileTime = NX,
		ValuesAtCompileTime = NY
	};
	typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

	int m_inputs, m_values;

	Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
	Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

	int inputs() const { return m_inputs; }
	int values() const { return m_values; }

};

class Residual :public MatchEdges, public Functor<double> {
public:
	double d_step;
	double Points=0;

	int res(const Eigen::VectorXf &params, Eigen::VectorXf &fvec) const{
		double params_array[6];
		params_array[0] = params[0];
		params_array[1] = params[1];
		params_array[2] = params[2];
		params_array[3] = params[3];
		params_array[4] = params[4];
		params_array[5] = params[5];
		fvec(0) = hausdorffDistance(params_array);

		return 0;
	}
	int operator() (const Eigen::VectorXf &params, Eigen::VectorXf &fvec) const {

		res(params, fvec);
		std::cout << "Hausdorff_distance " << fvec[0] << endl;
		return 0;
	}
	int df(const Eigen::VectorXf &params, Eigen::MatrixXf &fjac) const
	{
		for (int i = 0; i < 6; i++) {
			fjac(i,0) = partDerCentre(params,i);
		}
		
		return 0;
	}
	double partDerCentre(const Eigen::VectorXf& params,int i_th)const {
		//利用centre法则求解偏导，d_params代表移动的某方向正小量，例如(0,0,0,0,0,0.01)
		double params_array_l[6], params_array_r[6];
		for (int i = 0; i < 6; i++) {
			if (i == i_th) {
				params_array_l[i] = params[i];
				params_array_r[i] = params[i];
			}
			else {
				params_array_l[i] = params[i] - d_step;
				params_array_r[i] = params[i]+ d_step;
			}
		}

		double d = (hausdorffDistance(params_array_r) - hausdorffDistance(params_array_l)) / (d_step*2);
		cout << "'der " << d << endl;
		return (d);
	}

	int inputs() const { return 6; } //六个参数
	int values() const { return 1; } //数据个数
	Residual(Mat* cam_img_input, double d_step_input) :MatchEdges(cam_img_input) { d_step = d_step_input; };
	
};
 void solve(double* final_result) {


	 Residual residual(cam_img,0.1);
	// Eigen::NumericalDiff<Residual> numDiff(residual);
	 Eigen::LevenbergMarquardt<Residual, float> lm(residual);
	 Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(var);
	 std::cout << status << endl;
	 std::cout << lm.iter << std::endl;

	 std::cout << "var that minimizes the function: " << var << std::endl;
	

	 std::cout << "The final position and rotation are:  x:" << var(0) << " y: " << var(1) << " z: " << var(2) << " x_deg:" << var(3) << " y_deg:" << var(4) << " z_deg:" << var(5) << std::endl;

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
