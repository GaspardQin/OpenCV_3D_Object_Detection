
#include "MatchEdges.h"
using namespace std;
using namespace cv;

void MatchEdges::getModelImg(const double *var, Mat& model_canny_img) const {
	pos_model_set[0] = var[0];
	pos_model_set[1] = var[1];
	pos_model_set[2] = var[2]; //z为负值
	//rotate_degree_set[0] = var[3];
	//rotate_degree_set[1] = var[4];
	//rotate_degree_set[2] = var[5];
	quat_set[0] = var[3];
	quat_set[1] = var[4];
	quat_set[2] = var[5];
	quat_set[3] = sqrt(1 - quat_set[0] * quat_set[0] - quat_set[1] * quat_set[1] - quat_set[2]* quat_set[2]);
	ResetEvent(sentModelEvent);
	SetEvent(readModelEvent);
	
	WaitForSingleObject(sentModelEvent, INFINITE);
	//cv::flip(readSrcImg, readSrcImg, 0);
	Mat model_canny_img_pre;
	Canny(readSrcImg, model_canny_img, 50, 200);
}

const void MatchEdges::drawPoints(Mat &img, std::vector<Point2f> points, const Scalar& color)const {
	int i;
	for (i = 0; i < points.size(); i++) {
		circle(img, points[i], 5, color);
	}
}
const void MatchEdges::debugShowMatch(std::vector<Point2f> model_corners, std::vector<Point2f> cam_corners)const {
	// var 是位置、姿态参数，是一个大小为6的数组
	Mat cam_src_color = imread("./model/sample.jpg");
	Mat back_ground = cam_src_color.clone();

	int i;
	for (i = 0; i < model_corners.size(); i++) {
		Scalar color = Scalar(int(rand() * 255 / RAND_MAX), int(rand() * 255 / RAND_MAX), int(rand() * 255 / RAND_MAX));
		circle(back_ground, model_corners[i], 5, color);
		circle(back_ground, cam_corners[i], 5, color);
	}
	//drawPoints(back_ground, model_corners, Scalar(200, 150, 50));
	//drawPoints(back_ground, cam_corners, Scalar(50, 200, 50));
	imshow("debugShowMatchPoints_RANSAC", back_ground);
	
}

double MatchEdges::DTmatchHelp(Mat cam_DT, Mat model_canny_img, double k_l, double k_u) const {
	//DistanceTransform Match
	vector<double> dist;
	for (int i = 0; i < model_canny_img.size().height; i++) {
		for (int j = 0; j < model_canny_img.size().width; j++) {
			if (model_canny_img.at<uchar>(i,j) > 0){
				dist.push_back(cam_DT.at<float>(i,j));
			}
		}
	}
	sort(dist.begin(), dist.end());
	double sum = 0;
	for (int i = floor(k_u*dist.size()) - 1; i > floor(k_l*dist.size()); i--) {
		sum += dist[i];
	}
	std::cout << "max distance :" << dist[floor(k_u*dist.size()) - 1] << std::endl;
	return sum;
}
void MatchEdges::DT(Mat cam_img_, Mat &cam_DT_) const {
	bitwise_not(cam_img_, cam_img_);
	distanceTransform(cam_img_,cam_DT_, CV_DIST_L1, 3, CV_32F);
}
double MatchEdges::DTmatch(double* var, double k_l, double k_u) const {
	Mat model_canny_img;
	getModelImg(var, model_canny_img);
	return DTmatchHelp(cam_DT, model_canny_img, k_l, k_u);
}
double MatchEdges::DTmatchPyramid(double* var, int level, double k_l, double k_u) const {
	Mat model_canny_img;
	getModelImg(var, model_canny_img);
	//pyrDown(model_canny_img, model_canny_img);
	binaryZoomOut(model_canny_img, model_canny_img, 1/pow(2,level));
	return DTmatchHelp(cam_DT_pyramid[level], model_canny_img, k_l, k_u);
}
void MatchEdges::binaryZoomOut(Mat input_img, Mat &output_img, double f)const {
	resize(input_img, output_img, Size(0, 0), f, f, INTER_AREA);
	threshold(output_img, output_img, 20, 255, THRESH_BINARY);
}