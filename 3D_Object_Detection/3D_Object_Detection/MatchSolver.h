//�����װ�� dlib �ṩ�ķ��������Ż�����
#pragma once
#ifndef MATCH_SOLVER
#define MATCH_SOLVER
//#include "LM.cpp"
#include <MatchEdges.h>
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <dlib\optimization.h>
//using namespace dlib;

static double rho_quat = 500;//��Ԫ�������Ż�ʱ�Ŵ��ϵ�����ھ������ʱӦ��С��ͬϵ��

class MatchSolver{
public:

	typedef dlib::matrix<double, 6, 1> column_vector;
	typedef dlib::matrix<double, 1, 6> row_vector;
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
	
	class  CostFactorPyramid :public MatchEdges {
	public:
		typedef MatchSolver::column_vector column_vector;
		typedef dlib::matrix<double> general_matrix;
		Mat cam_img;
		int level;
		
		void setLevel(int level_) {
			level = level_; //���벻ͬ���Ż��׶ε�ʱ����Ҫ����level
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
			//	debugShowMatch(params_array);
			//	waitKey(10);
			iteral_count = iteral_count +1;
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
			// var ��λ�á���̬��������һ����СΪ6������
			
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
	
	
	void MatchSolver::dynamicBound(column_vector& lower_bound, column_vector& up_bound, column_vector var, double f) {
		lower_bound = var(0) - 20 * f, var(1) - 20 * f, var(2) - 15 * f, std::min(var(3) - 50 * f, -1.0*rho_quat), std::min(var(4) - 50 * f, -1.0*rho_quat), std::min(var(5) - 50 * f, -1.0*rho_quat);

		up_bound = var(0) + 20 * f, var(1) + 20 * f, var(2) + 15 * f, std::max(var(3) + 50 * f, 1.0*rho_quat), std::max(var(4) + 50 * f, 1.0*rho_quat), std::max(var(5) + 50 * f, 1.0*rho_quat);

	}
	void solve(double* final_result) {

		CostFactorPyramid cost_factor_pyramid(cam_img,4);

		double epsilon_final; 
		double var_double[6];

		var(3) *= rho_quat;
		var(4) *= rho_quat;
		var(5) *= rho_quat;
		cost_factor_pyramid.setLevel(4);
		
			column_vector lower_bound, up_bound;
			dynamicBound(lower_bound, up_bound, var, 1.0);
			find_min_bobyqa(cost_factor_pyramid,
				var,
				8,    // number of interpolation points
				lower_bound,  // lower bound constraint
				up_bound,   // upper bound constraint
				9,    // initial trust region radius
				0.01,  // stopping trust region radius
				500    // max number of objective function evaluations
			);
			double cost_left = cost_factor_pyramid(var);
			double ff = 1;
		while (cost_left > 1) {
			cost_factor_pyramid.setLevel(3);
			ff /= 2;
			dynamicBound(lower_bound, up_bound, var, ff);
			find_min_bobyqa(cost_factor_pyramid,
				var,
				8,    // number of interpolation points
				lower_bound,  // lower bound constraint
				up_bound,   // upper bound constraint
				9 *ff,    // initial trust region radius
				0.01,  // stopping trust region radius
				1000    // max number of objective function evaluations
			);
			cost_left = cost_factor_pyramid(var);
		}
		
		cout << "test_function solution:\n" << var << endl;
		cout << "total iteral times: " << iteral_count << endl;

		std::cout << "The final position and rotation are:  x: " << var(0) << " y: " << var(1) << " z: " << var(2) << " x_deg:" << var(3) << " y_deg:" << var(4) << " z_deg:" << var(5) << std::endl;
		
		final_result[0] = var(0);
		final_result[1] = var(1);
		final_result[2] = var(2);
		final_result[3] = var(3)/ rho_quat;
		final_result[4] = var(4)/ rho_quat;
		final_result[5] = var(5)/ rho_quat;
	};

private:
	
	cv::Mat  cam_img;
};




#endif
