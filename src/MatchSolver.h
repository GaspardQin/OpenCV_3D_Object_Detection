//该类封装了 dlib 提供的非线性最优化方法
#pragma once
#ifndef MATCH_SOLVER
#define MATCH_SOLVER
//#include "LM.cpp"
#include "MatchEdges.h"

class  CostFactor :public MatchEdges {
private:
	int option;
public:
	CostFactor(int option_)
		: MatchEdges() {
		option = option_;
	};

	double operator ()(
		int* params
		) const {
		return calculateDTfactor(params);
	};
	double calculateDTfactor(
		const int* params_array
	)const {

		double dist;
		switch (option)
		{
		case MODEL_CANNY_CAM_DT_ONLINE: dist = MatchOnline_modelCannycamDT(params_array, 0.3, 0.8); break;
		case MODEL_CANNY_CAM_DT_OFFLINE: dist = MatchOffline_modelCannycamDT(params_array, 0.3, 0.8); break;
		default:
			cout << "option input is not valide" << endl;
			break;
		}

		cout << "params input: x: " << params_array[0] << " y: " << params_array[1] << " z: " << params_array[2] << " x_deg: " << params_array[3] << " y_deg: " << params_array[4] << " z_deg: " << params_array[5] << endl;
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
	const void debugShowMatch(const int* var)const {
		// var 是位置、姿态参数，是一个大小为6的数组

		Mat back_ground = cam_img_src.clone();

		getModelImgUchar(var);
		for (int i = 0; i < back_ground.rows; i++)
		{
			for (int j = 0; j < back_ground.cols; j++)
			{
				if (readSrcImg.at<uchar>(i, j)>0) {
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
