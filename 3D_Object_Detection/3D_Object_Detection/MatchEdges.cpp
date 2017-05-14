
#include "MatchEdges.h"
using namespace std;
using namespace cv;
void MatchEdges::camCornerDect() {
	goodFeaturesToTrack(cam_img, cam_corners, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k); //未选择感兴趣区域  
}
void MatchEdges::modelCornerDect(const double *var, vector<Point2f> &model_corners) {
	Mat model_canny_img;
	getModelImg(var, model_canny_img);
	goodFeaturesToTrack(model_canny_img, model_corners, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k); //未选择感兴趣区域  
}
void MatchEdges::getModelImg(const double *var, Mat& model_canny_img) const {
	pos_model_set[0] = var[0];
	pos_model_set[1] = var[1];
	pos_model_set[2] = var[2]; //z为负值
	rotate_degree_set[0] = var[3];
	rotate_degree_set[1] = var[4];
	rotate_degree_set[2] = var[5];

	ResetEvent(sentModelEvent);
	SetEvent(readModelEvent);
	
	WaitForSingleObject(sentModelEvent, INFINITE);
	//cv::flip(readSrcImg, readSrcImg, 0);
	Mat model_canny_img_pre;
	Canny(readSrcImg, model_canny_img, 50, 200);
}

double MatchEdges::hausdorffDistance(const double *var) const {
	//待优化能量函数
	Mat model_canny_img;
	vector<Point2f> model_corners;
	getModelImg(var, model_canny_img);
	goodFeaturesToTrack(model_canny_img, model_corners, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k); //未选择感兴趣区域  
	//double dist = hausdorff_ptr->computeDistance(cam_corners, model_corners);
	double dist = hausdorffDistanceManuel(cam_corners, model_corners,0.8);
	return (dist);
	//微调位置，姿态使partial Hausdorff distance(系数0.6)最小（或者模板匹配）
}
double MatchEdges::hausdorffDistanceManuel(const std::vector<Point2f> &cam_corners, const std::vector<Point2f> &model_corners,double k) const{
	vector<double> distVec;
	double min_dist=1000, temp_dist;
	for (int i = 0; i < cam_corners.size(); i++) {
		min_dist = 1000;
		for (int j = 0; j < model_corners.size(); j++) {
			temp_dist =sqrt( pow(cam_corners[i].x - model_corners[j].x, 2) + pow(cam_corners[i].y - model_corners[j].y, 2));
			if (temp_dist < min_dist) {
				min_dist = temp_dist;
			}
		}
		distVec.push_back(min_dist);
	}
	sort(distVec.begin(), distVec.end());
	return distVec[floor(k*distVec.size()-1)];
}
double MatchEdges::hausdorffDistancePur(const double *var) const {
	Mat model_canny_img;
	getModelImg(var, model_canny_img);
	vector<Point2i> model_all_points;
	getAllPointsFromCanny(model_canny_img, model_all_points);
	double dist = hausdorff_ptr->computeDistance(cam_all_points, model_all_points);

	return (dist);
}
void MatchEdges::camAllPoints() {
	getAllPointsFromCanny(cam_img, cam_all_points);

}
void MatchEdges::getAllPointsFromCanny(Mat& model_canny_img, vector<Point2i>& model_all_points) const {
	for (int i = 0; i < model_canny_img.rows; i++)
	{
		for (int j = 0; j < model_canny_img.cols; j++)
		{
			if (model_canny_img.at<uchar>(i, j)>0) {
				model_all_points.push_back(Point2i(i,j));
			}
		}
	}
}

