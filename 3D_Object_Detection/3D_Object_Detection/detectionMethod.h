#pragma once
#include "thread_variables.h"
#include "PSO.h"
using namespace std;
using namespace cv;
class PosDetection {
public:
	std::vector<cv::Mat> model_ini_Canny_imgs;//存储预先生成的模型2D边缘图
	double deg_estimated[3]; double pixel_pos_estimated[2]; double scale_ratio_estimated;
	cv::Mat cam_canny_img, cam_src;
	int min_index;
	double var_best[6]; //最佳微调量
	template<typename T> int findMinIndex(std::vector<T>& src);
	template<typename T> int findMaxIndex(std::vector<T>& src);
	template<typename T> int maxElement(T* src, int size);
	void getInitialModelBuffer();
	void initialization();
	double huMatchFromCannyImg(int index);
	void PosDetection::centerPosEstimation(Mat &model_img, Mat &cam_img);
	void huCoarseDetection();
	double curveEstimation(double x1, double y1, double x2, double y2, double x3, double y3);
	void shi_TomasiDetection();


	PosDetection(double camera_z_set_input, double coarse_precision_x_deg_input, double coarse_precision_y_deg_input, double coarse_precision_z_deg_input, double x_deg_l_input, double x_deg_h_input, double y_deg_l_input, double y_deg_h_input, double z_deg_l_input, double z_deg_h_input) {
		camera_z_set_input = camera_z_set;
		precision_deg_x = coarse_precision_x_deg_input;
		precision_deg_y = coarse_precision_y_deg_input;
		precision_deg_z = coarse_precision_z_deg_input;
		deg_x_l = x_deg_l_input;
		deg_x_h = x_deg_h_input;
		deg_y_l = y_deg_l_input;
		deg_y_h = y_deg_h_input;
		deg_z_l = z_deg_l_input;
		deg_z_h = z_deg_h_input;
	}
private:
	double camera_z_set;
	double precision_deg_x;
	double precision_deg_y;
	double precision_deg_z;
	double deg_x_l;
	double deg_x_h;
	double deg_y_l;
	double deg_y_h;
	double deg_z_l;
	double deg_z_h;

};