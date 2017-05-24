#pragma once
#include "thread_variables.h"
//#include "MatchSolver.h"
#include "PSO.h"
using namespace cv;
using namespace std;
class DetectionMethod {
public:
	std::vector<cv::Mat> model_ini_Canny_imgs;//存储预先生成的模型2D边缘图
	double pixel_pos_estimated[2];
	double pos_estimated[3]; double quat_estimated[3];
	cv::Mat cam_canny_img, cam_src, cam_src_color;
	bool buffer_is_created = false;
	int min_index;
	template<typename T> int findMinIndex(std::vector<T>& src);
	template<typename T> int findMaxIndex(std::vector<T>& src);
	template<typename T> int maxElement(T* src, int size);
	template<typename T> int minElement(T* src, int size);

	void initialization();

	void DT_solve_with_powell(double* output_best);
	void DT_solve_with_PSO(double * output_best);
	void debugShowContours(int canny_index, std::vector<std::vector<Point>> *cam_contours, int cam_index, std::vector<std::vector<Point>> *model_contours, int model_index);
	void debugShowMatch(double* var);
	void drawPoints(Mat &img,std::vector<Point2f> points, const Scalar& color);

	DetectionMethod() {};
	

};