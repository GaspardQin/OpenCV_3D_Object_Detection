//该类封装了 dlib 提供的非线性最优化方法
#pragma once
#ifndef MATCH_SOLVER
#define MATCH_SOLVER
#include "LM.cpp"
#include <MatchEdges.h>
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <dlib\optimization.h>
//using namespace dlib;


class MatchSolver{
public:
	//typedef matrix<double, 2, 1> input_vector;
	//typedef matrix<double, 6, 1> parameter_vector;
	//typedef matrix<double, 1, 1> output_vector;
	typedef dlib::matrix<double, 6, 1> column_vector;
	typedef dlib::matrix<double, 1, 6> row_vector;
	column_vector var;
	//parameter_vector var;
	//std::vector<std::pair<input_vector, double> > data_samples;
	void setIniVar(double x, double y, double z, double x_deg, double y_deg, double z_deg) {
		var(0) = x;
		var(1) = y;
		var(2) = z;
		var(3) = x_deg;
		var(4) = y_deg;
		var(5) = z_deg;
	};
	MatchSolver(Mat & cam_img_input) { cam_img = cam_img_input; };
	
	class  CostFactor :public MatchEdges {
	public:
		typedef MatchSolver::column_vector column_vector;
		typedef dlib::matrix<double> general_matrix;
		Mat cam_img;

		CostFactor(const Mat &cam_img_input) : MatchEdges(cam_img_input) {
			cam_img = cam_img_input;
			cam_src_color = imread("./model/sample.jpg");
		}
		double calculateFactor (
			const column_vector& params
			) const {
			double params_array[6];
			params_array[0] = params(0);
			params_array[1] = params(1);
			params_array[2] = params(2);
			params_array[3] = params(3);
			params_array[4] = params(4);
			params_array[5] = params(5);
			double dist = hausdorffDistance(params_array);
			cout << "params input: x: " << params(0) << " y: " << params(1) << " z: " << params(2) << " x_deg: " << params(3) << " y_deg: " << params(4) << " z_deg: " << params(5) << endl;
			cout << "hausdorffDistance iteral " << dist << endl;
			debugShowMatch(params_array);
			waitKey(10);
			return dist;
		}

		Mat cam_src_color;
		const Mat cam_canny_img;
		const void drawPoints(Mat &img, std::vector<Point2f> points, const Scalar& color)const {
			int i;
			for (i = 0; i < points.size(); i++) {
				circle(img, points[i], 5, color);
			}
		}
		const void debugShowMatch(const double* var)const {
			// var 是位置、姿态参数，是一个大小为6的数组
			
			Mat back_ground = cam_src_color.clone();
			MatchEdges matchEdgesForShow(cam_img);
//
//			std::vector<Point2f> model_corners;
	//		matchEdgesForShow.modelCornerDect(var, model_corners);
//			drawPoints(back_ground, matchEdgesForShow.cam_corners, Scalar(255, 0, 0));
	//		drawPoints(back_ground, model_corners, Scalar(0, 255, 0));
	//		imshow("debugShowMatchPoints", back_ground);
//
			Mat back_ground2 = cam_src_color.clone();
			Mat model_canny_img_src;
			matchEdgesForShow.getModelImg(var, model_canny_img_src);
			for (int i = 0; i < back_ground2.rows; i++)
			{
				for (int j = 0; j < back_ground2.cols; j++)
				{
					if (model_canny_img_src.at<uchar>(i, j)>0) {
						back_ground2.at<Vec3b>(i, j)[0] = 200; //Blue;
						back_ground2.at<Vec3b>(i, j)[1] = 100; //g;
						back_ground2.at<Vec3b>(i, j)[2] = 0; //r;
					}
				}
			}
			imshow("debugShowMatchImgs", back_ground2);
		}
	};
	class CostDerivative :public CostFactor {
	public:

		double d_step_pos,d_step_deg;
		CostDerivative(double d_step_pos_input,double d_step_deg_input, Mat &cam_img_input):CostFactor(cam_img_input) {
			d_step_pos = d_step_pos_input;
			d_step_deg = d_step_deg_input;

		}
		const row_vector calculateDer (
			const column_vector& params
			) const {
			row_vector der;//梯度向量
			column_vector small_var;//微小变量
			
			
			small_var = d_step_pos, 0, 0, 0, 0, 0;
			der(0) = partDerForward(params, small_var);

			small_var = 0, d_step_pos, 0, 0, 0, 0;
			der(1) = partDerForward(params, small_var);

			small_var = 0, 0, d_step_pos, 0, 0, 0;
			der(2) = partDerForward(params, small_var);

			small_var = 0, 0, 0, d_step_deg, 0, 0;
			der(3) = partDerForward(params, small_var);

			small_var = 0, 0, 0, 0, d_step_deg, 0;
			der(4) = partDerForward(params, small_var);

			small_var = 0, 0, 0, 0, 0, d_step_deg;
			der(5) = partDerForward(params, small_var);
			cout << "der: x: " << der(0) << " y: " << der(1) << " z: " << der(2) << " x_deg: " << der(3) << " y_deg: " << der(4) << " z_deg: " << der(5) << endl;
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
			return ((hausdorffDistance(params_array_r) - hausdorffDistance(params_array_l)) / length(d_params)/2);
		}
		double partDerForward(const parameter_vector& params, const parameter_vector& d_params)const {
			//利用centre法则求解偏导，d_params代表移动的某方向正小量，例如(0,0,0,0,0,0.01)
			double params_array_l[6], params_array_r[6];

			params_array_l[0] = params(0);
			params_array_l[1] = params(1);
			params_array_l[2] = params(2);
			params_array_l[3] = params(3);
			params_array_l[4] = params(4);
			params_array_l[5] = params(5);

			params_array_r[0] = params(0) + d_params(0);
			params_array_r[1] = params(1) + d_params(1);
			params_array_r[2] = params(2) + d_params(2);
			params_array_r[3] = params(3) + d_params(3);
			params_array_r[4] = params(4) + d_params(4);
			params_array_r[5] = params(5) + d_params(5);
			double d_r = hausdorffDistance(params_array_r);
			double d_l = hausdorffDistance(params_array_l);
			double d = (d_r - d_l) / length(d_params);
			cout << "'der " << d << endl;
			return (d);
		}
	};
	
	
	void solve(double* final_result) {

		//	find_min(cg_search_strategy(),  // Use BFGS search algorithm
	 //   	gradient_norm_stop_strategy(1e-3), // Stop when the change in rosen() is less than 1e-7
		//	CostFactor(cam_img), CostDerivative(0.1,0.01,cam_img), var, 0);
		// Once the function ends the starting_point vector will contain the optimum point 
		// of (1,1).
		//
		
	//	column_vector lower_bound, up_bound;
	//	lower_bound = -250, -250, -200, -30, -30, -30;
	//	up_bound = 250, 250, 200, 30,30, 30;
	//	find_min_bobyqa(CostFactor(cam_img),
	//		var,
	//		25,    // number of interpolation points
	//		lower_bound,  // lower bound constraint
	//		up_bound,   // upper bound constraint
	//		20,    // initial trust region radius
	//		1e-3,  // stopping trust region radius
	//		100    // max number of objective function evaluations
	//	);
		CostFactor cost_factor(cam_img);
		CostDerivative cost_der(1, 0.1, cam_img);
		LM<CostFactor, CostDerivative> lm(cost_factor, cost_der,var,1,2,1e-7);
		double epsilon_final; bool flag;
		flag = lm.LM_solver(var, epsilon_final);
		if (flag == false) {
			cout << "Optimization failed." << endl;
		}
		cout << "test_function solution:\n" << var << endl;
		

		std::cout << "The final position and rotation are:  x: " << var(0) << " y: " << var(1) << " z: " << var(2) << " x_deg:" << var(3) << " y_deg:" << var(4) << " z_deg:" << var(5) << std::endl;
		
		final_result[0] = var(0);
		final_result[1] = var(1);
		final_result[2] = var(2);
		final_result[3] = var(3);
		final_result[4] = var(4);
		final_result[5] = var(5);
	};


/*
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
		debugShowMatch(params_array);
		waitKey(100);
		return dist;
	}
	Mat cam_src_color;
	const void drawPoints(Mat &img, std::vector<Point2f> points, const Scalar& color)const {
		int i;
		for (i = 0; i < points.size(); i++) {
			circle(img, points[i], 5, color);
		}
	}
	const void debugShowMatch(const double* var)const {
		// var 是位置、姿态参数，是一个大小为6的数组
		
		Mat back_ground = cam_src_color.clone();
		MatchEdges matchEdgesForShow(cam_img);
		
			std::vector<Point2f> model_corners;
			matchEdgesForShow.modelCornerDect(var, model_corners);
			drawPoints(back_ground, matchEdgesForShow.cam_corners, Scalar(255, 0, 0));
			drawPoints(back_ground, model_corners, Scalar(0, 255, 0));
			imshow("debugShowMatchPoints", back_ground);
		
		Mat back_ground2 = cam_src_color.clone();
		Mat model_canny_img_src;
		matchEdgesForShow.getModelImg(var, model_canny_img_src);
		for (int i = 0; i < back_ground2.rows; i++)
		{
			for (int j = 0; j < back_ground2.cols; j++)
			{
				if (model_canny_img_src.at<uchar>(i, j)>0) {
					back_ground2.at<Vec3b>(i, j)[0] = 200; //Blue;
					back_ground2.at<Vec3b>(i, j)[1] = 100; //g;
					back_ground2.at<Vec3b>(i, j)[2] = 0; //r;
				}
			}
		}
		imshow("debugShowMatchImgs", back_ground2);
	}
	residual(Mat cam_img_input) :MatchEdges(cam_img_input) { cam_src_color = imread("./model/sample.jpg"); };

};
class CostDerivative :public residual {
public:

	double d_step;
	CostDerivative(double d_step_input, Mat cam_img_input) :residual(cam_img_input) {
		d_step = d_step_input;
	}
	const parameter_vector operator() (
		const std::pair<input_vector, double>& data,
		const parameter_vector& params
		) const {
		parameter_vector der;//梯度向量
		parameter_vector small_var;//微小变量


		small_var = d_step, 0, 0, 0, 0, 0;
		der(0) = partDerForward(params, small_var);

		small_var = 0, d_step, 0, 0, 0, 0;
		der(1) = partDerForward(params, small_var);

		small_var = 0, 0, d_step*10, 0, 0, 0;
		der(2) = partDerForward(params, small_var);

		small_var = 0, 0, 0, d_step/10, 0, 0;
		der(3) = partDerForward(params, small_var);

		small_var = 0, 0, 0, 0, d_step/10, 0;
		der(4) = partDerForward(params, small_var);

		small_var = 0, 0, 0, 0, 0, d_step/10;
		der(5) = partDerForward(params, small_var);
		
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
	double partDerForward(const parameter_vector& params, const parameter_vector& d_params)const {
		//利用centre法则求解偏导，d_params代表移动的某方向正小量，例如(0,0,0,0,0,0.01)
		double params_array_l[6], params_array_r[6];

		params_array_l[0] = params(0) ;
		params_array_l[1] = params(1) ;
		params_array_l[2] = params(2) ;
		params_array_l[3] = params(3) ;
		params_array_l[4] = params(4) ;
		params_array_l[5] = params(5) ;

		params_array_r[0] = params(0) + d_params(0);
		params_array_r[1] = params(1) + d_params(1);
		params_array_r[2] = params(2) + d_params(2);
		params_array_r[3] = params(3) + d_params(3);
		params_array_r[4] = params(4) + d_params(4);
		params_array_r[5] = params(5) + d_params(5);
		double d_r = hausdorffDistance(params_array_r);
		double d_l = hausdorffDistance(params_array_l);
		double d = (d_r - d_l) / length(d_params);
		cout << "'der " << d << endl;
		return (d);
	}
};

 void solve(double* final_result) {
	 parameter_vector var_ = var;
	 dlib::solve_least_squares(gradient_norm_stop_strategy(1e-5),//.be_verbose(),
		 residual(cam_img),
		 CostDerivative(1,cam_img),
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
 */
private:
	
	cv::Mat  cam_img;
};




#endif
