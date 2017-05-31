#pragma once
#include "thread_variables.h"
using namespace std;
using namespace cv;
class MatchEdges {
private:
	boost::shared_array<Mat> model_DT_imgs;
	boost::shared_array<std::vector<Point2i>> model_points_vec_array;
	boost::shared_array<int> init_buffer_l_boundary;
	boost::shared_array<int> init_buffer_r_boundary;
	boost::shared_array<int> init_buffer_precision;
	boost::shared_array<int> init_buffer_count_for_levels;
	vector<Point2i> cam_canny_points;
public:
	Mat cam_img_debug;
	Mat cam_img, cam_img_pyramid_1, cam_img_pyramid_2, cam_DT, cam_DT_pyramid_1, cam_DT_pyramid_2;
	std::vector<Mat> cam_DT_pyramid;
	int levelMax;

	int dim = 6;//总共六维
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
	MatchEdges(const Mat &cam_img_input, boost::shared_array<Mat>& model_DT_imgs_, boost::shared_array<int>& init_buffer_l_boundary_, boost::shared_array<int>& init_buffer_r_boundary_, boost::shared_array<int>& init_buffer_precision_, boost::shared_array<int>& init_buffer_count_for_levels_) {
		//for offline OpenGL Only
		init_buffer_l_boundary = init_buffer_l_boundary_;
		init_buffer_r_boundary = init_buffer_r_boundary_;
		init_buffer_precision = init_buffer_precision_;
		init_buffer_count_for_levels = init_buffer_count_for_levels_;
		getCannyPoints(cam_img_input, cam_canny_points);
	
		model_DT_imgs = model_DT_imgs_;
	};
	MatchEdges(const Mat &cam_img_input, boost::shared_array<std::vector<Point2i>>& model_points_vec_array_, boost::shared_array<int>& init_buffer_l_boundary_, boost::shared_array<int>& init_buffer_r_boundary_, boost::shared_array<int>& init_buffer_precision_, boost::shared_array<int>& init_buffer_count_for_levels_) {
		//for offline OpenGL Only
		init_buffer_l_boundary = init_buffer_l_boundary_;
		init_buffer_r_boundary = init_buffer_r_boundary_;
		init_buffer_precision = init_buffer_precision_;
		init_buffer_count_for_levels = init_buffer_count_for_levels_;
		DT(cam_img_input, cam_DT);
		model_points_vec_array = model_points_vec_array_;
	};
	MatchEdges(boost::shared_array<int>& init_buffer_l_boundary_, boost::shared_array<int>& init_buffer_r_boundary_, boost::shared_array<int>& init_buffer_precision_, boost::shared_array<int>& init_buffer_count_for_levels_) {
		//for offline OpenGL buffer creating Only
		init_buffer_l_boundary = init_buffer_l_boundary_;
		init_buffer_r_boundary = init_buffer_r_boundary_;
		init_buffer_precision = init_buffer_precision_;
		init_buffer_count_for_levels = init_buffer_count_for_levels_;
	};
	void getCannyPoints(const Mat & cam_canny_img, vector<Point2i>& points)const;
	void getModelImgUchar(const int* var, Mat& model_canny_img) const;
	template <typename T> void getModelImg(const T *var, Mat& model_canny_img) const;
	double DTmatchHelp(Mat cam_DT, Mat model_canny_img, double k_l, double k_u) const;
	double modelDTcamCannyMatchHelp(Mat model_DT, vector<Point2i> cam_canny_points, double k_l, double k_u) const;
	template <typename T> double MatchEdges::modelDTcamCannyROIMatchHelp(Mat model_DT, T* var, vector<Point2i> cam_canny_points, double k_l, double k_u) const;
	double DTmatchPyramid(double *var,int level, double k_l, double k_u) const;
	template <typename T> double modelDTcamCannyMatch(T* var, double k_l, double k_u) const;
	template <typename T> double modelDTcamCannyROIMatch(T* var, double k_l, double k_u) const;
	void DT(Mat cam_img, Mat &cam_DT) const;
	void MatchEdges::DT_L1(Mat cam_img_, Mat &cam_DT_) const;
	void binaryZoomOut(Mat input_img, Mat &output_img, double f)const;
	int getIndex(int* var) const {
		return getIndex(var[0], var[1], var[2], var[3], var[4], var[5]);
	}
	int getIndex(double* var) const {
		return getIndex(int(var[0]), int(var[1]), int(var[2]), int(var[3]), int(var[4]), int(var[5]));
	}
	int getIndex(const int x, const int y, const int z, const int deg_x, const int deg_y, const int deg_z) const{
		//得到array中对应的index
		int index, x_count, y_count, z_count, deg_x_count, deg_y_count, deg_z_count;
		if ((x - init_buffer_l_boundary[0]) % init_buffer_precision[0] > 0) {
			while (1) {
				std::cout << "Error: wrong Index of x !" << std::endl;
			}
		}
		if ((y - init_buffer_l_boundary[1]) % init_buffer_precision[1] > 0) {
			while (1) {
				std::cout << "Error: wrong Index of y !" << std::endl;
			}
		}
		if ((z - init_buffer_l_boundary[2]) % init_buffer_precision[2] > 0) {
			while (1) {
				std::cout << "Error: wrong Index of z !" << std::endl;
			}
		}
		if ((deg_x - init_buffer_l_boundary[3]) % init_buffer_precision[3] > 0) {
			while (1) {
				std::cout << "Error: wrong Index of deg_x !" << std::endl;
			}
		}
		if ((deg_y - init_buffer_l_boundary[4]) % init_buffer_precision[4] > 0) {
			while (1) {
				std::cout << "Error: wrong Index of deg_y !" << std::endl;
			}
		}
		if ((deg_z - init_buffer_l_boundary[5]) % init_buffer_precision[5] > 0) {
			while (1) {
				std::cout << "Error: wrong Index of deg_z !" << std::endl;
			}
		}


		x_count = (x - init_buffer_l_boundary[0]) / init_buffer_precision[0];//从0开始数
		y_count = (y - init_buffer_l_boundary[1]) / init_buffer_precision[1];
		z_count = (z - init_buffer_l_boundary[2]) / init_buffer_precision[2];
		deg_x_count = (deg_x - init_buffer_l_boundary[3]) / init_buffer_precision[3];
		deg_y_count = (deg_y - init_buffer_l_boundary[4]) / init_buffer_precision[4];
		deg_z_count = (deg_z - init_buffer_l_boundary[5]) / init_buffer_precision[5];

		/*
		index = x_count * (init_buffer_num[1] * init_buffer_num[2] * init_buffer_num[3] * init_buffer_num[4] * init_buffer_num[5])
		+ y_count * (init_buffer_num[2] * init_buffer_num[3] * init_buffer_num[4] * init_buffer_num[5])
		+ z_count * (init_buffer_num[3] * init_buffer_num[4] * init_buffer_num[5])
		+ deg_x_count * (init_buffer_num[4] * init_buffer_num[5])
		+ deg_y_count*init_buffer_num[5]
		+ deg_z_count;
		*/

		//避免重复运算乘法
		index = x_count * init_buffer_count_for_levels[1] + y_count * init_buffer_count_for_levels[2] + z_count * init_buffer_count_for_levels[3]
			+ deg_x_count *init_buffer_count_for_levels[4] + deg_y_count*init_buffer_count_for_levels[5] + deg_z_count;
		return index;

	}
	void getROI(Mat img_input, Mat& ROI_output, double x, double y, double z)const;
	void getROIrect(double x, double y, double z, int* output_array) const;
	template <typename T> double modelCannycamDT_Match(T* var, double k_l, double k_u) const;
};

template <typename T> void MatchEdges::getModelImg(const T* var, Mat& model_canny_img) const {
	boost::lock_guard<boost::mutex> lock(gl_mutex); //保证每个时刻只有一个线程能与OpenGL通信
	pos_model_set[0] = var[0];
	pos_model_set[1] = var[1];
	pos_model_set[2] = var[2]; //z为负值
	rotate_degree_set[0] = var[3];
	rotate_degree_set[1] = var[4];
	rotate_degree_set[2] = var[5];
	//quat_set.x = var[3];
	//quat_set.y = var[4];
	//quat_set.z = var[5];
	//quat_set.w = sqrt(1 - quat_set.x * quat_set.x - quat_set.y * quat_set.y - quat_set.z* quat_set.z);
	ResetEvent(sentModelEvent);
	SetEvent(readModelEvent);

	WaitForSingleObject(sentModelEvent, INFINITE);
	//cv::flip(readSrcImg, readSrcImg, 0);
	Mat model_canny_img_pre;
	Canny(readSrcImg, model_canny_img, 50, 200);
	//double a;
	//a = (1.0 / 255.0);
	model_canny_img.convertTo(model_canny_img, CV_32FC1);
};
template <typename T> double MatchEdges::modelDTcamCannyROIMatchHelp(Mat model_DT, T* var, vector<Point2i> cam_canny_points, double k_l, double k_u) const {
	/*
	//**************for debug
	Mat model_canny_img,model_DT_img, model_ROI;
	getModelImgUchar(var, model_canny_img);

	DT_L1(model_canny_img, model_DT_img);
	getROI(model_DT_img, model_ROI, var[0], var[1], var[2]);
	Mat comp;
	comp = (model_ROI == model_DT);

	Mat cam_debug;
	getROI(cam_img_debug, cam_debug, var[0], var[1], var[2]);
	//***************
	*/
	
	
	vector<double> dist;
	int rect_pixel[4];//column_left_pixel,row_top_pixel,,column_right_pixel,row_bottom_pixel
	getROIrect(double(var[0]), double(var[1]), double(var[2]), rect_pixel);
	int col_temp, row_temp;
	for (int i = 0; i < cam_canny_points.size(); i++) {
		 row_temp= cam_canny_points[i].x;
		 col_temp = cam_canny_points[i].y;
		if (col_temp < rect_pixel[0]) col_temp = rect_pixel[0];
		else if (col_temp > rect_pixel[2]) col_temp = rect_pixel[2];

		if (row_temp < rect_pixel[1]) row_temp = rect_pixel[1];
		else if (row_temp > rect_pixel[3]) row_temp = rect_pixel[3];

		//int debug = model_DT.at<uchar>(row_temp - rect_pixel[1], col_temp - rect_pixel[0]);
		dist.push_back(model_DT.at<uchar>(row_temp - rect_pixel[1], col_temp - rect_pixel[0]));//at<>(row,column), 与普通points相反
	}
	sort(dist.begin(), dist.end());

	double sum = 0;
	for (int i = floor(k_u*dist.size()) - 1; i > floor(k_l*dist.size()); i--) {
		sum += dist[i];
	}
	std::cout << "max distance :" << dist[floor(k_u*dist.size()) - 1] << std::endl;
	return sum*dist[floor(k_u*dist.size()) - 1];

}


template <typename T> double MatchEdges::modelDTcamCannyMatch(T* var, double k_l, double k_u) const {
	return modelDTcamCannyMatchHelp(model_DT_imgs[getIndex(var)], cam_canny_points, k_l, k_u);
}
template <typename T> double MatchEdges::modelDTcamCannyROIMatch(T* var, double k_l, double k_u) const {
	return modelDTcamCannyROIMatchHelp<T>(model_DT_imgs[getIndex(var)], var, cam_canny_points, k_l, k_u);
}
template <typename T> double MatchEdges::modelCannycamDT_Match(T* var, double k_l, double k_u) const {
	vector<double> dist;
	int col_temp, row_temp;
	int debug_ = getIndex(var);
	int debug = model_points_vec_array[getIndex(var)].size();
	for (int i = 0; i < model_points_vec_array[getIndex(var)].size(); i++) {

		dist.push_back(cam_DT.at<float>(model_points_vec_array[getIndex(var)][i].x, model_points_vec_array[getIndex(var)][i].y));
	}
	sort(dist.begin(), dist.end());

	double sum = 0;
	for (int i = floor(k_u*dist.size()) - 1; i > floor(k_l*dist.size()); i--) {
		sum += dist[i];
	}
	std::cout << "max distance :" << dist[floor(k_u*dist.size()) - 1] << std::endl;
	return sum*dist[floor(k_u*dist.size()) - 1];

}