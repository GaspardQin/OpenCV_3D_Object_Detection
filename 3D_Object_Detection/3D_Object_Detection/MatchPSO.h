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

	//�ǵ��������
	// Shi-Tomasi�Ĳ�������  
	vector<Point2f> cam_corners_PSO;
	int maxCorners = 30;    //�ǵ���������ֵ  
	double qualityLevel = 0.01;
	double minDistance = 10;
	int blockSize = 3;
	bool useHarrisDetector = false; //��ʹ��Harris����㷨  
	double k = 0.04;

	cv::Ptr <cv::HausdorffDistanceExtractor> hausdorff_ptr = cv::createHausdorffDistanceExtractor();


	int dim = 6;//�ܹ���ά
	MatchPSO(Mat *cam_img_input, double* deg_input, double *pixel_pos_input, double scale_ratio_estimated_before_input) {
		cam_img = cam_img_input;
		deg_estimated_before = deg_input;//3ά
		pixel_pos_estimated_before = pixel_pos_input;//2ά
		scale_ratio_estimated_before = scale_ratio_estimated_before_input;//1ά
		hausdorff_ptr->setRankProportion(0.6);
		camCornerDect();
	};
	double hausdorffCost(double *var);//x����ά����
	void camCornerDect();
	void getModelImg(double *var, Mat& model_canny_img);
};