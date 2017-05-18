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
	int maxCorners = 30;    //�ǵ���������ֵ  
	double qualityLevel = 0.001;
	double minDistance = 5;
	int blockSize = 3;
	bool useHarrisDetector = false; //��ʹ��Harris����㷨  
	double k = 0.04;
	std::vector<Point2f> cam_corners_nearest;
	cv::Ptr <cv::HausdorffDistanceExtractor> hausdorff_ptr = cv::createHausdorffDistanceExtractor();


	int dim = 6;//�ܹ���ά
	MatchEdges(const Mat &cam_img_input) {
		cam_img = cam_img_input;
		hausdorff_ptr->setRankProportion(0.6);
		hausdorff_ptr->setDistanceFlag(NORM_L2);
		camCornerDect();
		camAllPoints();
	};
	double hausdorffDistance(const double *var)const;//x����ά����
	double MatchEdges::hausdorffDistance(const double *var, std::vector<Point2f> &cam_corners_nearest)const;

	double hausdorffDistancePur(const double *var) const;//x����ά����

	void camCornerDect();//��ȡ�����ǵ㣬���hausdorff�����ٶ�
	void modelCornerDect(const double *var, std::vector<Point2f> &model_corners)const;
	void getModelImg(const double *var, Mat& model_canny_img) const;
	void getAllPointsFromCanny(Mat& model_canny_img, std::vector<Point2i>& model_all_points) const;
	void camAllPoints();
	std::vector<Point2i> cam_all_points;
	double hausdorffDistanceManuel(const std::vector<Point2f> &cam_corners, const std::vector<Point2f> &model_corners, double k) const;
	double hausdorffDistanceManuelSum(const std::vector<Point2f> &cam_corners, const std::vector<Point2f> &model_corners, double k1, double k2) const;
	double hausdorffDistanceManuelSum(const std::vector<Point2f> &cam_corners, const std::vector<Point2f> &model_corners, std::vector<Point2f> &cam_corners_nearest, double k1, double k2)const;
	void RANSACfilter(double * var);
	const void debugShowMatch(std::vector<Point2f> model_corners, std::vector<Point2f> cam_corners)const;
	const void drawPoints(Mat &img, std::vector<Point2f> points, const Scalar& color)const;
	double RANSACfilterHaursdoffDistance(double * var) const;
};