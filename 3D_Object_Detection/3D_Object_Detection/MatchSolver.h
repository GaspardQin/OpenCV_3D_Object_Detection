//该类封装了 dlib 提供的非线性最优化方法
#pragma once
#ifndef MATCH_SOLVER
#define MATCH_SOLVER
#include "LM.cpp"
#include <MatchEdges.h>
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <dlib\optimization.h>
//using namespace dlib;

static double rho_quat = 500;//四元数在最优化时放大的系数，在具体计算时应缩小相同系数
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
	void setIniVar(double x, double y, double z, double quat_x, double quat_y, double quat_z) {
		var(0) = x;
		var(1) = y;
		var(2) = z;
		var(3) = quat_x;
		var(4) = quat_y;
		var(5) = quat_z;
	};
	MatchSolver(Mat & cam_img_input){
		cam_img = cam_img_input;
	};
	
	class  CostFactor :public MatchEdges {
	public:
		typedef MatchSolver::column_vector column_vector;
		typedef dlib::matrix<double> general_matrix;
		Mat cam_img;

		CostFactor(const Mat &cam_img_input) : MatchEdges(cam_img_input) {
			cam_img = cam_img_input;
			cam_src_color = imread("./model/sample.jpg");
		}

		double operator ()(
			const column_vector& params
			) const {
			return calculateDTfactor(params);
		};

		double calculateDTfactor(
			const column_vector& params
		)const {
			double params_array[6];
			params_array[0] = params(0);
			params_array[1] = params(1);
			params_array[2] = params(2);
			params_array[3] = params(3) / rho_quat;
			params_array[4] = params(4) / rho_quat;
			params_array[5] = params(5) / rho_quat;
			double dist = DTmatch(params_array,0.3,0.8);


			glm::quat q;
			q.x = params_array[3];
			q.y = params_array[4];
			q.z = params_array[5];
			q.w = sqrt(1 - q.x*q.x - q.y*q.y - q.z*q.z);
			glm::vec3 euler = glm::degrees(glm::eulerAngles(q));

			cout << "params input: x: " << params(0) << " y: " << params(1) << " z: " << params(2) << " x_deg: " << euler.x << " y_deg: " << euler.y << " z_deg: " << euler.z << endl;
			cout << "DT score iteral " << dist << endl;
			debugShowMatch(params_array);
			waitKey(10);
			return dist;
		}
		double calculateFactor(
			const column_vector& params
			)const {
			double params_array[6];
			params_array[0] = params(0);
			params_array[1] = params(1);
			params_array[2] = params(2);
			params_array[3] = params(3)/ rho_quat;
			params_array[4] = params(4)/ rho_quat;
			params_array[5] = params(5)/ rho_quat;
			double dist = hausdorffDistance(params_array);

			glm::quat q;
			q.x = params_array[3];
			q.y = params_array[4];
			q.z = params_array[5];
			q.w = sqrt(1 - q.x*q.x - q.y*q.y - q.z*q.z);
			glm::vec3 euler = glm::degrees(glm::eulerAngles(q));

			cout << "params input: x: " << params(0) << " y: " << params(1) << " z: " << params(2) << " x_deg: " << euler[0]<< " y_deg: " << euler.y << " z_deg: " << euler.z << endl;
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
			

		//	std::vector<Point2f> model_corners;
			//matchEdgesForShow.modelCornerDect(var, model_corners);
			//drawPoints(back_ground, matchEdgesForShow.cam_corners, Scalar(255, 0, 0));
			//drawPoints(back_ground, model_corners, Scalar(0, 255, 0));
			//imshow("debugShowMatchPoints", back_ground);

			Mat back_ground2 = cam_src_color.clone();
			Mat model_canny_img_src;
			getModelImg(var, model_canny_img_src);
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
	class  CostFactorPyramid1 :public CostFactor {
	public:
		CostFactorPyramid1(const Mat &cam_img_input) : CostFactor(cam_img_input) {
		}
		double operator ()(
			const column_vector& params
			) const {
			return calculateDTfactorPyramid1(params);
		};

		double calculateDTfactorPyramid1(
			const column_vector& params
		)const {
			double params_array[6];
			params_array[0] = params(0);
			params_array[1] = params(1);
			params_array[2] = params(2);
			params_array[3] = params(3) / rho_quat;
			params_array[4] = params(4) / rho_quat;
			params_array[5] = params(5) / rho_quat;
			double dist = DTmatchPyramid1(params_array, 0.3, 0.8);


			glm::quat q;
			q.x = params_array[3];
			q.y = params_array[4];
			q.z = params_array[5];
			q.w = sqrt(1 - q.x*q.x - q.y*q.y - q.z*q.z);
			glm::vec3 euler = glm::degrees(glm::eulerAngles(q));

			cout << "params input: x: " << params(0) << " y: " << params(1) << " z: " << params(2) << " x_deg: " << euler.x << " y_deg: " << euler.y << " z_deg: " << euler.z << endl;
			cout << "DTpyramid1 score iteral " << dist << endl;
		//	debugShowMatch(params_array);
		//	waitKey(10);
			return dist;
		}
	};
	class  CostFactorPyramid2 :public CostFactor {
	public:
		CostFactorPyramid2(const Mat &cam_img_input) : CostFactor(cam_img_input) {
		}
		double operator ()(
			const column_vector& params
			) const {
			return calculateDTfactorPyramid2(params);
		};

		double calculateDTfactorPyramid2(
			const column_vector& params
		)const {
			double params_array[6];
			params_array[0] = params(0);
			params_array[1] = params(1);
			params_array[2] = params(2);
			params_array[3] = params(3) / rho_quat;
			params_array[4] = params(4) / rho_quat;
			params_array[5] = params(5) / rho_quat;
			double dist = DTmatchPyramid2(params_array, 0.3, 0.8);


			glm::quat q;
			q.x = params_array[3];
			q.y = params_array[4];
			q.z = params_array[5];
			q.w = sqrt(1 - q.x*q.x - q.y*q.y - q.z*q.z);
			glm::vec3 euler = glm::degrees(glm::eulerAngles(q));

			cout << "params input: x: " << params(0) << " y: " << params(1) << " z: " << params(2) << " x_deg: " << euler.x << " y_deg: " << euler.y << " z_deg: " << euler.z << endl;
			cout << "DTpyramid2 score iteral " << dist << endl;
			//debugShowMatch(params_array);
		//	waitKey(10);
			return dist;
		}
	};
	class CostFactorPrecise :public CostFactor {
	public:
		CostFactorPrecise(Mat &cam_img_input) :CostFactor(cam_img_input) {		};
		double operator ()(
			const column_vector& params
			) const {
			return calculateFactor(params);
		};
		double calculateFactor(
			const column_vector& params
		)const {
			double params_array[6];
			params_array[0] = params(0);
			params_array[1] = params(1);
			params_array[2] = params(2);
			params_array[3] = params(3)/ rho_quat;
			params_array[4] = params(4)/ rho_quat;
			params_array[5] = params(5)/ rho_quat;
			double dist = RANSACfilterDistance(params_array);
			cout << "params input: x: " << params(0) << " y: " << params(1) << " z: " << params(2) << " x_deg: " << params(3) << " y_deg: " << params(4) << " z_deg: " << params(5) << endl;
			cout << "RANSACDistance iteral " << dist << endl;
			debugShowMatch(params_array);
			waitKey(10);
			return dist;
		}
	};

	class CostDerivative :public CostFactor {
	public:

		double d_step_pos,d_step_deg;
		CostDerivative(double d_step_pos_input,double d_step_deg_input, Mat &cam_img_input):CostFactor(cam_img_input) {
			d_step_pos = d_step_pos_input;
			d_step_deg = d_step_deg_input;

		}
		const row_vector operator()(
			const column_vector& params
			)  const {
			return calculateDer(params);
		};
		const row_vector calculateDer (
			const column_vector& params
			)  const {
			row_vector der;//梯度向量
			column_vector small_var;//微小变量
			
			
			small_var = d_step_pos, 0, 0, 0, 0, 0;
			der(0) = partDerCentre(params, small_var);

			small_var = 0, d_step_pos, 0, 0, 0, 0;
			der(1) = partDerCentre(params, small_var);

			small_var = 0, 0, 2*d_step_pos, 0, 0, 0;
			der(2) = partDerCentre(params, small_var)*5;//z轴导数过小

			small_var = 0, 0, 0, d_step_deg, 0, 0;
			der(3) = partDerCentre(params, small_var);

			small_var = 0, 0, 0, 0, d_step_deg, 0;
			der(4) = partDerCentre(params, small_var);

			small_var = 0, 0, 0, 0, 0, d_step_deg;
			der(5) = partDerCentre(params, small_var);
			cout << "der: x: " << der(0) << " y: " << der(1) << " z: " << der(2) << " x_deg: " << der(3) << " y_deg: " << der(4) << " z_deg: " << der(5) << endl;
			return der;
		}
		const row_vector calculateDerPrecise(
			const column_vector& params
		)  const {
			row_vector der;//梯度向量
			column_vector small_var;//微小变量
		

			small_var = d_step_pos/10, 0, 0, 0, 0, 0;
			der(0) = partDerCentre(params, small_var);

			small_var = 0, d_step_pos/10, 0, 0, 0, 0;
			der(1) = partDerCentre(params, small_var);

			small_var = 0, 0, d_step_pos/10, 0, 0, 0;
			der(2) = partDerCentre(params, small_var);

			small_var = 0, 0, 0, d_step_deg/10, 0, 0;
			der(3) = partDerCentre(params, small_var);

			small_var = 0, 0, 0, 0, d_step_deg/10, 0;
			der(4) = partDerCentre(params, small_var);

			small_var = 0, 0, 0, 0, 0, d_step_deg/10;
			der(5) = partDerCentre(params, small_var);
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
			double d_r = hausdorffDistance(params_array_r);
			double d_l = hausdorffDistance(params_array_l);
			double d = (d_r - d_l) / length(d_params)/2;
			while (d< 0.0001 && d>-0.0001) {
				d = partDerCentre(params, 2 * d_params); //防止导数为0，无法计算LM的情况
			}
			return d;
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
			while (d< 0.0001 && d>-0.0001) {
				d = partDerForward(params, 2 * d_params); //防止导数为0，无法计算LM的情况
			}
			cout << "'der " << d << endl;
			return (d);
		}
	};
	
	void MatchSolver::dynamicBound(parameter_vector& lower_bound, parameter_vector& up_bound, parameter_delta_vector var, double f) {
		lower_bound = var(0) - 20 * f, var(1) - 20 * f, var(2) - 15 * f, std::min(var(3) - 50 * f, -1.0*rho_quat), std::min(var(4) - 50 * f, -1.0*rho_quat), std::min(var(5) - 50 * f, -1.0*rho_quat);

		up_bound = var(0) + 20 * f, var(1) + 20 * f, var(2) + 15 * f, std::max(var(3) + 50 * f, 1.0*rho_quat), std::max(var(4) + 50 * f, 1.0*rho_quat), std::max(var(5) + 50 * f, 1.0*rho_quat);

	}
	void solve(double* final_result) {

		//	find_min(cg_search_strategy(),  // Use BFGS search algorithm
	 //   	gradient_norm_stop_strategy(1e-3), // Stop when the change in rosen() is less than 1e-7
		//	CostFactor(cam_img), CostDerivative(0.1,0.01,cam_img), var, 0);
		// Once the function ends the starting_point vector will contain the optimum point 
		// of (1,1).
		//
		

		CostFactor cost_factor(cam_img);

		CostFactorPyramid1 cost_factor_pyramid1(cam_img);
		CostFactorPyramid2 cost_factor_pyramid2(cam_img);
		// cost_der(0.1, 0.05, cam_img);
		//LM<CostFactor, CostDerivative> lm(cost_factor, cost_der,var,0.00025,5,1e-7);
		double epsilon_final; bool flag;
		//flag = lm.LM_solver(var, epsilon_final);
		double var_double[6];
		var_double[0] = var(0);
		var_double[1] = var(1);
		var_double[2] = var(2);
		var_double[3] = var(3);
		var_double[4] = var(4);
		var_double[5] = var(5);

	//	cost_factor.RANSACfilter(var_double);

		var(0) = var_double[0];
		var(1) = var_double[1];
		var(2) = var_double[2];
		var(3) = var_double[3];
		var(4) = var_double[4];
		var(5) = var_double[5];


		var(3) *= rho_quat;
		var(4) *= rho_quat;
		var(5) *= rho_quat;
		flag = true;
			column_vector lower_bound, up_bound;
			dynamicBound(lower_bound, up_bound, var, 1.0);
			find_min_bobyqa(cost_factor_pyramid2,
				var,
				8,    // number of interpolation points
				lower_bound,  // lower bound constraint
				up_bound,   // upper bound constraint
				9,    // initial trust region radius
				0.01,  // stopping trust region radius
				500    // max number of objective function evaluations
			);
			double cost_left = cost_factor_pyramid2(var);
			double ff = 1;
		while (cost_left > 1) {
			ff /= 2;
			dynamicBound(lower_bound, up_bound, var, ff);
			find_min_bobyqa(cost_factor_pyramid2,
				var,
				8,    // number of interpolation points
				lower_bound,  // lower bound constraint
				up_bound,   // upper bound constraint
				9 *ff,    // initial trust region radius
				0.01,  // stopping trust region radius
				1000    // max number of objective function evaluations
			);
			cost_left = cost_factor_pyramid2(var);
		}
		/*
		column_vector lower_bound_precise, up_bound_precise;
		lower_bound_precise = var(0) - 5, var(1) - 5, var(2) - 5, std::min(var(3) - 10, -1.0), std::min(var(4) - 10, -1.0*rho_quat), std::min(var(5) - 10, -1.0*rho_quat);

		up_bound_precise = var(0) + 5, var(1) + 5, var(2) + 5, std::max(var(3) + 10, 1.0*rho_quat), std::max(var(4) + 10, 1.0*rho_quat), std::max(var(5) + 10, 1.0*rho_quat);

		find_min_bobyqa(CostFactorPrecise(cam_img),
			var,
			9,    // number of interpolation points
			lower_bound_precise,  // lower bound constraint
			up_bound_precise,   // upper bound constraint
			4,    // initial trust region radius
			0.001,  // stopping trust region radius
			300    // max number of objective function evaluations
		);
		var(3) /= rho_quat;
		var(4) /= rho_quat;
		var(5) /= rho_quat;
		//double var_double[6];
		var_double[0] = var(0);
		var_double[1] = var(1);
		var_double[2] = var(2);
		var_double[3] = var(3);
		var_double[4] = var(4);
		var_double[5] = var(5);


	//	cost_factor.calculateFactor(var);
	*/
		if (flag == false) {var(0) = var_double[0];
			cout << "Optimization failed." << endl;
		}
		cout << "test_function solution:\n" << var << endl;
		

		std::cout << "The final position and rotation are:  x: " << var(0) << " y: " << var(1) << " z: " << var(2) << " x_deg:" << var(3) << " y_deg:" << var(4) << " z_deg:" << var(5) << std::endl;
		
		final_result[0] = var(0);
		final_result[1] = var(1);
		final_result[2] = var(2);
		final_result[3] = var(3)/ rho_quat;
		final_result[4] = var(4)/ rho_quat;
		final_result[5] = var(5)/ rho_quat;
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
