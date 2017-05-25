//该类封装了 dlib 提供的非线性最优化方法
#pragma once
#ifndef MATCH_SOLVER
#define MATCH_SOLVER
//#include "LM.cpp"
#include <MatchEdges.h>
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <dlib\optimization.h>
//using namespace dlib;

static double rho_quat = 500;//四元数在最优化时放大的系数，在具体计算时应缩小相同系数
typedef dlib::matrix<double, 6, 1> column_vector;
typedef dlib::matrix<double, 1, 6> row_vector;
class  CostFactorPyramid :public MatchEdges {
public:
	//typedef MatchSolver::column_vector column_vector;
	typedef dlib::matrix<double> general_matrix;
	Mat cam_img;
	int level;

	void setLevel(int level_) {
		level = level_; //进入不同层优化阶段的时候需要调整level
	}
	CostFactorPyramid(const Mat &cam_img_input, int level_max_) : MatchEdges(cam_img_input, level_max_) {
		level = level_max_;
		
		cam_img = cam_img_input;
		cam_src_color = imread("./model/sample.jpg");
		
	}

	double operator ()(
		const column_vector& params
		) const {
		return calculateDTfactorPyramid(params);
	};
	double calculateDTfactorPyramid(double* params_array) {
		params_array[3] = params_array[3] / rho_quat;
		params_array[4] = params_array[4] / rho_quat;
		params_array[5] = params_array[5] / rho_quat;
		double dist = DTmatchPyramid(params_array, level, 0.3, 0.8);
		glm::quat q;
		q.x = params_array[3];
		q.y = params_array[4];
		q.z = params_array[5];
		q.w = sqrt(1 - q.x*q.x - q.y*q.y - q.z*q.z);
		glm::vec3 euler = glm::degrees(glm::eulerAngles(q));

		cout << "params input: x: " << params_array[0] << " y: " << params_array[1] << " z: " << params_array[2] << " x_deg: " << euler.x << " y_deg: " << euler.y << " z_deg: " << euler.z << endl;
		cout << "DTpyramid1 score iteral " << dist << endl;
		cout << "level" << level << endl;
		//	debugShowMatch(params_array);
		//	waitKey(10);
		iteral_count = iteral_count + 1;
		return dist;

	}
	double calculateDTfactorPyramid(
		const column_vector& params
	)const {
		double params_array[6];
		params_array[0] = params(0);
		params_array[1] = params(1);
		params_array[2] = params(2);
		params_array[3] = params(3) / rho_quat;
		params_array[4] = params(4) / rho_quat;
		params_array[5] = params(5) / rho_quat;
		double dist = DTmatchPyramid(params_array, level, 0.3, 0.8);


		glm::quat q;
		q.x = params_array[3];
		q.y = params_array[4];
		q.z = params_array[5];
		q.w = sqrt(1 - q.x*q.x - q.y*q.y - q.z*q.z);
		glm::vec3 euler = glm::degrees(glm::eulerAngles(q));

		cout << "params input: x: " << params(0) << " y: " << params(1) << " z: " << params(2) << " x_deg: " << euler.x << " y_deg: " << euler.y << " z_deg: " << euler.z << endl;
		cout << "DTpyramid1 score iteral " << dist << endl;
		cout << "level" << level << endl;
		//	debugShowMatch(params_array);
		//	waitKey(10);
		iteral_count = iteral_count + 1;
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

		Mat model_canny_img_src;
		getModelImg(var, model_canny_img_src);
		for (int i = 0; i < back_ground.rows; i++)
		{
			for (int j = 0; j < back_ground.cols; j++)
			{
				if (model_canny_img_src.at<uchar>(i, j)>0) {
					back_ground.at<Vec3b>(i, j)[0] = 200; //Blue;
					back_ground.at<Vec3b>(i, j)[1] = 100; //g;
					back_ground.at<Vec3b>(i, j)[2] = 0; //r;
				}
			}
		}
		imshow("debugShowMatchImgs", back_ground);
	}
};
class MatchSolver{
public:

	
	
	column_vector var;

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
	
	
	
	
	void MatchSolver::dynamicBound(column_vector& lower_bound, column_vector& up_bound, column_vector var, double f) {
		lower_bound = var(0) - 20 * f, var(1) -20 * f, var(2) - 20 * f, std::max(var(3) - 20 * f, -1.0*rho_quat), std::max(var(4) - 20 * f, -1.0*rho_quat), std::max(var(5) - 20 * f, -1.0*rho_quat);

		up_bound = var(0) + 20 * f, var(1) + 20 * f, var(2) + 20 * f, std::min(var(3) + 20 * f, 1.0*rho_quat), std::min(var(4) + 20 * f, 1.0*rho_quat), std::min(var(5) + 20 * f, 1.0*rho_quat);

	}
	void solve(double* final_result) {
		int level_max = 4;
		CostFactorPyramid cost_factor_pyramid(cam_img, level_max);

		double epsilon_final; 
		double var_double[6];

		var(3) *= rho_quat;
		var(4) *= rho_quat;
		var(5) *= rho_quat;
		double ff = 1.0;
		double stop_trust_region = 0.01;
		double init_trust_region = 19;
		column_vector lower_bound, up_bound;
		for (int i = level_max; i >= 0; i--) {
			i = 0;
			cost_factor_pyramid.setLevel(i);

			dynamicBound(lower_bound, up_bound, var, ff);
			find_min_bobyqa(cost_factor_pyramid,
				var,
				18,    // number of interpolation points
				lower_bound,  // lower bound constraint
				up_bound,   // upper bound constraint
				init_trust_region,    // initial trust region radius
				stop_trust_region,  // stopping trust region radius
				500    // max number of objective function evaluations
			);

			////////如果陷入局部最小点，重新搜索
			double cost_left = cost_factor_pyramid(var);
			
			while (cost_left >= 50/(i+1)) {
				
				dynamicBound(lower_bound, up_bound, var, ff);
				find_min_bobyqa(cost_factor_pyramid,
					var,
					20,    // number of interpolation points
					lower_bound,  // lower bound constraint
					up_bound,   // upper bound constraint
					init_trust_region,    // initial trust region radius
					stop_trust_region,  // stopping trust region radius
					1000    // max number of objective function evaluations
				);
				cost_left = cost_factor_pyramid(var);
			}
			/////进入下一层高精度搜索
			ff *= 0.5;
			stop_trust_region *= 0.5;
			init_trust_region *=  0.5;
		}
	

		final_result[0] = var(0);
		final_result[1] = var(1);
		final_result[2] = var(2);
		final_result[3] = var(3) / rho_quat;
		final_result[4] = var(4) / rho_quat;
		final_result[5] = var(5) / rho_quat;
		cout << "test_function solution:\n" << var << endl;
		cout << "total iteral times: " << iteral_count << endl;

		std::cout << "The final position and rotation are:  x: " << var(0) << " y: " << var(1) << " z: " << var(2) << " x_deg:" << var(3) << " y_deg:" << var(4) << " z_deg:" << var(5) << std::endl;
		

	};

private:
	
	cv::Mat  cam_img;
};




#endif
