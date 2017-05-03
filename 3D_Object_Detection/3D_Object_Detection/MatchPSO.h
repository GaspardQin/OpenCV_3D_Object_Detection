#pragma once
#include "thread_variables.h"
using namespace std;
using namespace cv;
class MatchPSO {
public:
	Mat* cam_img;
	double* deg_estimated_before;
	double* pixel_pos_estimated_before;
	double scale_ratio_estimated_before;

	//角点参数设置
	// Shi-Tomasi的参数设置  
	vector<Point2f> cam_corners_PSO;
	int maxCorners = 30;    //角点个数的最大值  
	double qualityLevel = 0.01;
	double minDistance = 10;
	int blockSize = 3;
	bool useHarrisDetector = false; //不使用Harris检测算法  
	double k = 0.04;

	cv::Ptr <cv::HausdorffDistanceExtractor> hausdorff_ptr = cv::createHausdorffDistanceExtractor();


	int dim = 6;//总共六维
	MatchPSO(Mat *cam_img_input, double* deg_input, double *pixel_pos_input, double scale_ratio_estimated_before_input) {
		cam_img = cam_img_input;
		deg_estimated_before = deg_input;//3维
		pixel_pos_estimated_before = pixel_pos_input;//2维
		scale_ratio_estimated_before = scale_ratio_estimated_before_input;//1维
		hausdorff_ptr->setRankProportion(0.6);
		camCornerDect();
	};
	double hausdorffCost(double *var);//x是六维向量
	void camCornerDect();
	void getModelImg(double *var, Mat& model_canny_img);
};