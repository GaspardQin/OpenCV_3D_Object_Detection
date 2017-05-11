#pragma once
#include "thread_variables.h"
using namespace std;
using namespace cv;
class MatchEdges {
public:
	Mat* cam_img;

	//�ǵ��������
	// Shi-Tomasi�Ĳ�������  
	std::vector<Point2f> cam_corners;
	int maxCorners = 30;    //�ǵ���������ֵ  
	double qualityLevel = 0.01;
	double minDistance = 10;
	int blockSize = 3;
	bool useHarrisDetector = false; //��ʹ��Harris����㷨  
	double k = 0.04;

	cv::Ptr <cv::HausdorffDistanceExtractor> hausdorff_ptr = cv::createHausdorffDistanceExtractor();


	int dim = 6;//�ܹ���ά
	MatchEdges(Mat *cam_img_input) {
		cam_img = cam_img_input;
		hausdorff_ptr->setRankProportion(0.6);
		camCornerDect();
	};
	double hausdorffDistance(const double *var) const;//x����ά����
	void camCornerDect();//��ȡ�����ǵ㣬���hausdorff�����ٶ�
	void modelCornerDect(double *var, std::vector<Point2f> &model_corners);
	void getModelImg(const double *var, Mat& model_canny_img) const;
};