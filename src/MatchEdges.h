#pragma once
#include "thread_variables.h"
using namespace std;
using namespace cv;
class MatchEdges {
private:
	float* row_camDT_ptrs[int(WINDOW_HEIGHT)]; //cam_DT矩阵每一行第一个值的地址（为了取代at函数，做到对元素的快速访问）
public:
	Mat cam_img_debug;

	MatchEdges() {
		for (int i = 0; i < cam_DT.rows; i++) {
			row_camDT_ptrs[i] = cam_DT.ptr<float>(i);
		}
	};
	void getModelImgUchar(const int* var, Mat& model_canny_img) const;
	template <typename T> void getModelImg(const T *var, Mat& model_canny_img) const;
	double DTmatchHelp(Mat cam_DT, Mat model_canny_img, double k_l, double k_u) const;

	template <typename T> double modelDTcamCannyMatch(T* var, double k_l, double k_u) const;
	template <typename T> double modelDTcamCannyROIMatch(T* var, double k_l, double k_u) const;
	void DT(Mat cam_img, Mat &cam_DT) const;
	void MatchEdges::DT_L1(Mat cam_img_, Mat &cam_DT_) const;
	double DTmatchOnline(const int * var, double k_l, double k_u) const;
	void binaryZoomOut(Mat input_img, Mat &output_img, double f)const;
	void getROI(Mat img_input, Mat& ROI_output, double x, double y, double z)const;
	void getROIrect(double x, double y, double z, int* output_array) const;
	template <typename T> double modelCannycamDTMatch(T* var, double k_l, double k_u) const;
};

template <typename T> void MatchEdges::getModelImg(const T* var, Mat& model_canny_img) const {
	boost::lock_guard<boost::mutex> lock(gl_mutex); //保证每个时刻只有一个线程能与OpenGL通信
	pos_model_set[0] = var[0];
	pos_model_set[1] = var[1];
	pos_model_set[2] = var[2]; //z为负值
	rotate_degree_set[0] = var[3];
	rotate_degree_set[1] = var[4];
	rotate_degree_set[2] = var[5];

	ResetEvent(sentModelEvent);
	SetEvent(readModelEvent);

	WaitForSingleObject(sentModelEvent, INFINITE);
	cvtColor(readSrcImg, model_canny_img, CV_RGB2GRAY);
	model_canny_img.convertTo(model_canny_img, CV_32FC1);
};



template <typename T> double MatchEdges::modelDTcamCannyMatch(T* var, double k_l, double k_u) const {
	//需要重写
}
template <typename T> double MatchEdges::modelDTcamCannyROIMatch(T* var, double k_l, double k_u) const {
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

	Mat model_DT = model_offline_DT_imgs[discrete_info.getIndex(var)];
	vector<double> dist;
	int rect_pixel[4];//column_left_pixel,row_top_pixel,,column_right_pixel,row_bottom_pixel
	getROIrect(double(var[0]), double(var[1]), double(var[2]), rect_pixel);
	int col_temp, row_temp;
	for (int i = 0; i < cam_canny_points.size(); i++) {
		row_temp = cam_canny_points[i].x;
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
template <typename T> double MatchEdges::modelCannycamDTMatch(T* var, double k_l, double k_u) const {
	//检查cache中是否有保存
	int index = discrete_info.getIndex(var);
	if (cache_match[index] >= 0) { 
		boost::shared_lock<boost::shared_mutex> lock(cv_cache_mutex);//读取操作是采用读取锁（可多线程同时读取）
		return cache_match[index];
	}

	vector<double> dist;
	double temp;
	vector<Point2i> *  point_vec = &model_offline_canny_points[index];
/*
//****************************debug for show
	

	Mat back_ground = cam_img.clone();
	for (std::vector<Point2i>::iterator i = point_vec->begin(); i < point_vec->end(); i++)
	{

		back_ground.at<uchar>(i->y, i->x) = 150; //Blue;


	}

	imshow("debugShowMatchImgs", back_ground);
	waitKey(10);
//****************************
*/


	for (std::vector<Point2i>::iterator iter = point_vec->begin(); iter < point_vec->end(); iter++) {
		temp = row_camDT_ptrs[iter->y][iter->x];//findNonZero得到的Point x,y与Mat中的坐标相反
		if(temp > 0) dist.push_back(temp);
	}
	sort(dist.begin(), dist.end());

	double sum = 0;
	if (dist.size() == 0) dist.push_back(0);
	else {
		for (int i = floor(k_u*dist.size()) ; i > floor(k_l*dist.size()); i--) {
			sum += dist[i];
		}
	}

	std::cout << "max distance :" << dist[floor(k_u*dist.size())] << std::endl;
	boost::lock_guard<boost::shared_mutex> lock(cv_cache_mutex); //保证每个时刻只有一个线程能写入cache_match;
	cache_match[index] = sum; //存入cache；
	
	return sum;// *dist[floor(k_u*dist.size()) - 1];

}