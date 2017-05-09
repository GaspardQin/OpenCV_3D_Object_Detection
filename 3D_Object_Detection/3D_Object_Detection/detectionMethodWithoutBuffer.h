#pragma once
#include "thread_variables.h"
#include "MatchSolver.h"
using namespace std;
using namespace cv;
class DetectionMethod {
public:
	std::vector<cv::Mat> model_ini_Canny_imgs;//存储预先生成的模型2D边缘图
	double rotate_degree_estimated[3]; double pixel_pos_estimated[2];
	double pos_estimated[3];
	cv::Mat cam_canny_img, cam_src, cam_src_color;
	bool buffer_is_created = false;
	int min_index;
	template<typename T> int findMinIndex(std::vector<T>& src);
	template<typename T> int findMaxIndex(std::vector<T>& src);
	template<typename T> int maxElement(T* src, int size);
	template<typename T> int minElement(T* src, int size);

	void initialization();

	void shi_TomasiDetection(double* output_best);
	void debugShowContours(int canny_index, vector<vector<Point>> *cam_contours, int cam_index, vector<vector<Point>> *model_contours, int model_index);
	void debugShowMatch(double* var);
	void drawPoints(Mat &img, vector<Point2f> points, const Scalar& color);

	DetectionMethod() {};
	

};