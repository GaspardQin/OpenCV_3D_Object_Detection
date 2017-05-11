#pragma once
#include "thread_variables.h"
using namespace std;
using namespace cv;
class MatchEdges {
public:
	Mat* cam_img;

	//角点参数设置
	// Shi-Tomasi的参数设置  
	std::vector<Point2f> cam_corners;
	int maxCorners = 30;    //角点个数的最大值  
	double qualityLevel = 0.01;
	double minDistance = 10;
	int blockSize = 3;
	bool useHarrisDetector = false; //不使用Harris检测算法  
	double k = 0.04;

	cv::Ptr <cv::HausdorffDistanceExtractor> hausdorff_ptr = cv::createHausdorffDistanceExtractor();


	int dim = 6;//总共六维
	MatchEdges(Mat *cam_img_input) {
		cam_img = cam_img_input;
		hausdorff_ptr->setRankProportion(0.6);
		camCornerDect();
	};
	double hausdorffDistance(const double *var) const;//x是六维向量
	void camCornerDect();//提取轮廓角点，提高hausdorff计算速度
	void modelCornerDect(double *var, std::vector<Point2f> &model_corners);
	void getModelImg(const double *var, Mat& model_canny_img) const;
};