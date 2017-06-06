//该类封装了 dlib 提供的非线性最优化方法
#pragma once
#ifndef MATCH_SOLVER
#define MATCH_SOLVER
//#include "LM.cpp"
#include "MatchEdges.h"


class  CostFactorOnline :public MatchEdges {
public:
	CostFactorOnline()
		: MatchEdges() {	};

	double operator ()(
		int* params
		) const {
		return calculateDTfactorPyramid(params);
	};
	double calculateDTfactorPyramid(
		const int* params_array
	)const {

		double dist = DTmatchOnline(params_array, 0.3, 0.8);

		cout << "params input: x: " << params_array[0] << " y: " << params_array[1] << " z: " << params_array[2] << " x_deg: " << params_array[3]<< " y_deg: " << params_array[4] << " z_deg: " << params_array[5] << endl;
		cout << "DT score iteral " << dist << endl;
		//	debugShowMatch(params_array);
		//	waitKey(10);
		iteral_count = iteral_count + 1;
		return dist;
	}



	const void drawPoints(Mat &img, std::vector<Point2f> points, const Scalar& color)const {
		int i;
		for (i = 0; i < points.size(); i++) {
			circle(img, points[i], 5, color);
		}
	}
	const void debugShowMatch(const double* var)const {
		// var 是位置、姿态参数，是一个大小为6的数组

		Mat back_ground = cam_img_src.clone();

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

	CostFactorDT_Offline_modelCanny_camDT()
		: MatchEdges() {
	}

	double operator ()(
		const int* params
		) const {
		return calculateDTfactorPyramid(params);
	};

	double calculateDTfactorPyramid(
		const int* params_array
	)const {
		double dist = modelCannycamDTMatch(params_array, 0, 0.6);

		cout << "params input: x: " << params_array[0] << " y: " << params_array[1] << " z: " << params_array[2] << " x_deg: " << params_array[3] << " y_deg: " << params_array[4] << " z_deg: " << params_array[5] << endl;
		cout << "DT offline score iteral " << dist << endl;
		cout << "iteral_count" << iteral_count << endl;
		//	debugShowMatch(params_array);
		//	waitKey(10);
		iteral_count = iteral_count + 1;
		return dist;
	}



	const void drawPoints(Mat &img, std::vector<Point2f> points, const Scalar& color)const {
		int i;
		for (i = 0; i < points.size(); i++) {
			circle(img, points[i], 5, color);
		}
	}
	const void debugShowMatch(const double* var)const {
		// var 是位置、姿态参数，是一个大小为6的数组

		Mat back_ground = cam_img_src.clone();

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
