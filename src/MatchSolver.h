//该类封装了 dlib 提供的非线性最优化方法
#pragma once
#ifndef MATCH_SOLVER
#define MATCH_SOLVER
//#include "LM.cpp"
#include "MatchEdges.h"
#include <dlib\optimization.h>
//using namespace dlib;

static double rho_quat = 50;//四元数在最优化时放大的系数，在具体计算时应缩小相同系数
typedef dlib::matrix<double, 6, 1> column_vector;
typedef dlib::matrix<double, 1, 6> row_vector;

class  CostFactorOnlinePyramid :public MatchEdges {
public:
	//typedef MatchSolver::column_vector column_vector;
	typedef dlib::matrix<double> general_matrix;
	Mat cam_img;
	int level;

	void setLevel(int level_) {
		level = level_; //进入不同层优化阶段的时候需要调整level
	}
	CostFactorOnlinePyramid(const Mat &cam_img_input,int level_max_, boost::shared_array<int>& init_buffer_l_boundary_, boost::shared_array<int>& init_buffer_r_boundary_, boost::shared_array<int>& init_buffer_precision_, boost::shared_array<int>& init_buffer_count_for_levels_, boost::shared_array<double>& cache_match_)
		: MatchEdges(cam_img_input, init_buffer_l_boundary_, init_buffer_r_boundary_, init_buffer_precision_, init_buffer_count_for_levels_, cache_match_) {

		level = level_max_;
		
		cam_img = cam_img_input;
		cam_src_color = imread("../model/sample.bmp");
		
	}

	double operator ()(
		const column_vector& params
		) const {
		return calculateDTfactorPyramid(params);
	};
	double calculateDTfactorPyramid(
		const column_vector& params
	)const {
		int params_array[6];
		params_array[0] = params(0);
		params_array[1] = params(1);
		params_array[2] = params(2);
		params_array[3] = params(3);
		params_array[4] = params(4);
		params_array[5] = params(5);
		double dist = DTmatchOnlinePyramid(params_array, level, 0.3, 0.8);

		cout << "params input: x: " << params(0) << " y: " << params(1) << " z: " << params(2) << " x_deg: " << params(3)<< " y_deg: " << params(4) << " z_deg: " << params(5) << endl;
		cout << "DTpyramid1 score iteral " << dist*pow(2, level)*pow(2, level) << endl;
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
		getModelImg<double>(var, model_canny_img_src);
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

class  CostFactorDT_Offline_modelCanny_camDT :public MatchEdges {
public:
	//typedef MatchSolver::column_vector column_vector;
	typedef dlib::matrix<double> general_matrix;
	Mat cam_img;
	int level;

	void setLevel(int level_) {
		level = level_; //进入不同层优化阶段的时候需要调整level
	}
	CostFactorDT_Offline_modelCanny_camDT(const Mat &cam_img_input, boost::shared_array<std::vector<Point2i>>& model_points_vec_array, boost::shared_array<int>& init_buffer_l_boundary_, boost::shared_array<int>& init_buffer_r_boundary_, boost::shared_array<int>& init_buffer_precision_, boost::shared_array<int>& init_buffer_count_for_levels_, boost::shared_array<double>& cache_match_)
		: MatchEdges(cam_img_input, model_points_vec_array, init_buffer_l_boundary_, init_buffer_r_boundary_, init_buffer_precision_, init_buffer_count_for_levels_, cache_match_) {

		cam_img = cam_img_input;
		cam_src_color = imread("../model/sample.bmp");

	}

	double operator ()(
		const column_vector& params
		) const {
		return calculateDTfactorPyramid(params);
	};

	double calculateDTfactorPyramid(
		const column_vector& params
	)const {
		int params_array[6];
		params_array[0] = params(0);
		params_array[1] = params(1);
		params_array[2] = params(2);
		params_array[3] = params(3);
		params_array[4] = params(4);
		params_array[5] = params(5);
		double dist = modelCannycamDT_Match(params_array, 0, 0.6);

		cout << "params input: x: " << params(0) << " y: " << params(1) << " z: " << params(2) << " x_deg: " << params(3) << " y_deg: " << params(4) << " z_deg: " << params(5) << endl;
		cout << "DTpyramid offline score iteral " << dist*pow(2, level)*pow(2, level) << endl;
		cout << "level" << level << endl;
		cout << "iteral_count" << iteral_count << endl;
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
		getModelImg<double>(var, model_canny_img_src);
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




#endif
