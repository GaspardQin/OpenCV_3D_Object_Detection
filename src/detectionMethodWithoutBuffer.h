#pragma once

#ifndef  _DETECTION_METHOD_H_
#define _DETECTION_METHOD_H_

#include "thread_variables.h"
#include "MatchSolver.h"
#include "DE.h"
using namespace cv;
using namespace std;
class DetectionMethod {
private:
	int init_buffer_var[6];
	int init_buffer_precision[6];
	int init_buffer_delta[6];
	int init_buffer_l_boundary[6];
	int init_buffer_r_boundary[6];
	int init_buffer_num[6];//每个维度有多少个点
	int init_buffer_count_for_levels[6]; //参见getIndex()
	boost::shared_array<Mat> model_DT_imgs;
public:
	
	std::vector<cv::Mat> model_ini_Canny_imgs;//存储预先生成的模型2D边缘图
	double pixel_pos_estimated[2];
	double pos_estimated[3]; //double quat_estimated[3]; 
	double rotate_estimated[3];
	cv::Mat cam_canny_img, cam_src, cam_src_color;
	bool buffer_is_created = false;
	int min_index;
	template<typename T> int findMinIndex(std::vector<T>& src);
	template<typename T> int findMaxIndex(std::vector<T>& src);
	template<typename T> int maxElement(T* src, int size);
	template<typename T> int minElement(T* src, int size);
	void setBufferInitValue(int x_init, int y_init, int z_init, int deg_x, int deg_y, int deg_z);
	void setBufferPrecision(int x_precision, int y_precision, int z_precision, int x_deg_precsion, int y_deg_precision, int z_deg_precision);
	void setBufferBoundary(int delta_x, int delta_y, int delta_z, int delta_x_deg, int delta_y_deg, int delta_z_deg);
	int getIndex(int x, int y, int z, int deg_x, int deg_y, int deg_z); 
	int getIndex(int* var);

	void creatBuffer();
	void readBuffer();
	void arrayMatWrite(const string& filename, const Mat* matrices, const int array_size);
	void arrayMatRead(const string& filename, int array_size);
	void initialization();

	void DT_solve_with_powell(double* output_best);

	void DetectionMethod::DT_solve_with_DE(double * output_best);
	void debugShowContours(int canny_index, std::vector<std::vector<Point>> *cam_contours, int cam_index, std::vector<std::vector<Point>> *model_contours, int model_index);
	void debugShowMatch(double* var);
	void drawPoints(Mat &img,std::vector<Point2f> points, const Scalar& color);

	DetectionMethod() {};
	

};
#endif // ! _DETECTION_METHOD_H_
