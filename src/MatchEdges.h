#pragma once
#include "thread_variables.h"
using namespace std;
using namespace cv;
class MatchEdges {
public:
	Mat cam_img, cam_img_pyramid_1, cam_img_pyramid_2, cam_DT, cam_DT_pyramid_1, cam_DT_pyramid_2;
	std::vector<Mat> cam_DT_pyramid;
	int levelMax;

	int dim = 6;//×Ü¹²ÁùÎ¬
	MatchEdges(const Mat &cam_img_input,int levelMax_input) {
		cam_img = cam_img_input;
		levelMax = levelMax_input;
		for (int i = 0; i <= levelMax; i ++) {
			Mat cam_img_pyramid;
			Mat cam_DT;
			binaryZoomOut(cam_img, cam_img_pyramid, 1/std::pow(2,i));
			DT(cam_img_pyramid, cam_DT);
			cam_DT_pyramid.push_back(cam_DT);
		}
	};
	MatchEdges(int levelMax_input) {
		levelMax = levelMax_input;

	};
	void MatchEdges::getModelImg(const int* var, Mat& model_canny_img) const;
	void getModelImg(const double *var, Mat& model_canny_img) const;
	const void debugShowMatch(std::vector<Point2f> model_corners, std::vector<Point2f> cam_corners)const;
	const void drawPoints(Mat &img, std::vector<Point2f> points, const Scalar& color)const;
	double MatchEdges::DTmatchHelp(Mat cam_DT, Mat model_canny_img, double k_l, double k_u) const;
	double MatchEdges::DTmatchPyramid(double *var,int level, double k_l, double k_u) const;
	void MatchEdges::DT(Mat cam_img, Mat &cam_DT) const;
	void MatchEdges::binaryZoomOut(Mat input_img, Mat &output_img, double f)const;
};