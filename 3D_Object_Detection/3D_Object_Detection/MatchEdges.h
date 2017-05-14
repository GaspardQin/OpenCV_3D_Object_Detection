#pragma once
#include "thread_variables.h"
using namespace std;
using namespace cv;
class MatchEdges {
public:
	 Mat cam_img;

	//�ǵ��������
	// Shi-Tomasi�Ĳ�������  
	std::vector<Point2f> cam_corners;
	int maxCorners = 40;    //�ǵ���������ֵ  
	double qualityLevel = 0.01;
	double minDistance = 10;
	int blockSize = 3;
	bool useHarrisDetector = false; //��ʹ��Harris����㷨  
	double k = 0.04;

	cv::Ptr <cv::HausdorffDistanceExtractor> hausdorff_ptr = cv::createHausdorffDistanceExtractor();


	int dim = 6;//�ܹ���ά
	MatchEdges(const Mat &cam_img_input) {
		cam_img = cam_img_input;
		hausdorff_ptr->setRankProportion(1);
		hausdorff_ptr->setDistanceFlag(NORM_L2);
		camCornerDect();
		camAllPoints();
	};
	double hausdorffDistance(const double *var) const;//x����ά����
	double hausdorffDistancePur(const double *var) const;//x����ά����
	void camCornerDect();//��ȡ�����ǵ㣬���hausdorff�����ٶ�
	void modelCornerDect(const double *var, std::vector<Point2f> &model_corners);
	void getModelImg(const double *var, Mat& model_canny_img) const;
	void getAllPointsFromCanny(Mat& model_canny_img, std::vector<Point2i>& model_all_points) const;
	void camAllPoints();
	std::vector<Point2i> cam_all_points;
	double MatchEdges::hausdorffDistanceManuel(const std::vector<Point2f> &cam_corners, const std::vector<Point2f> &model_corners, double k) const;
};