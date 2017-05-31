
#include "MatchEdges.h"
using namespace std;
using namespace cv;
void MatchEdges::getROI(Mat img_input, Mat& ROI_output,double x, double y, double z) const{
	int pos_pixel[4];//column_left_pixel,row_top_pixel,,column_right_pixel,row_bottom_pixel
	getROIrect(x, y, z, pos_pixel);

	ROI_output = img_input(Range(pos_pixel[1],pos_pixel[3]),Range(pos_pixel[0], pos_pixel[2]));
}
void MatchEdges::getROIrect(double x, double y, double z,int* output_array) const {
	//确定左上角和右下角位置
	//若采用flip,则x_pixel应作一个关于中心轴的对称
	int column_left_pixel = round(WINDOW_WIDTH / 2 + x / (CCD_WIDTH / 2 / FOCAL_DISTANCE*(-z))*WINDOW_WIDTH - ROI_WIDTH / 2);//z为负数
	int row_top_pixel = round(WINDOW_HEIGHT / 2 + y / (CCD_WIDTH / 2 / FOCAL_DISTANCE*(-z) / WINDOW_WIDTH*WINDOW_HEIGHT)*WINDOW_HEIGHT - ROI_HEIGHT / 2);
	int column_right_pixel = column_left_pixel + ROI_WIDTH;
	int row_bottom_pixel = row_top_pixel + ROI_HEIGHT;

	//防止超出边界
	if (column_left_pixel < 0) column_left_pixel = 0;
	if (row_top_pixel < 0) row_top_pixel = 0;
	if (column_right_pixel >= WINDOW_WIDTH) column_right_pixel = WINDOW_WIDTH - 1;
	if (row_bottom_pixel >= WINDOW_HEIGHT) row_bottom_pixel = WINDOW_HEIGHT - 1;
	output_array[0] = column_left_pixel;
	output_array[1] = row_top_pixel;
	output_array[2] = column_right_pixel;
	output_array[3] = row_bottom_pixel;
}


void MatchEdges::getModelImgUchar(const int* var, Mat& model_canny_img) const {
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
}



double MatchEdges::modelDTcamCannyMatchHelp(Mat model_DT,vector<Point2i> cam_canny_points, double k_l, double k_u) const {
	vector<double> dist;
	for (int i = 0; i < cam_canny_points.size(); i++) {
		dist.push_back(model_DT.at<float>(cam_canny_points[i].x, cam_canny_points[i].y));
	}
	sort(dist.begin(), dist.end());
	
	double sum = 0;
	for (int i = floor(k_u*dist.size()) - 1; i > floor(k_l*dist.size()); i--) {
		sum += dist[i];
	}
	std::cout << "max distance :" << dist[floor(k_u*dist.size()) - 1] << std::endl;
	return sum*dist[floor(k_u*dist.size()) - 1];

}
double MatchEdges::DTmatchHelp(Mat cam_DT, Mat model_canny_img, double k_l, double k_u) const {
	//DistanceTransform Match
	vector<double> dist;
	/*
	for (int i = 0; i < model_canny_img.size().height; i++) {
		for (int j = 0; j < model_canny_img.size().width; j++) {
			if (model_canny_img.at<uchar>(i,j) > 0){
				dist2.push_back(cam_DT.at<uchar>(i,j));
			}
		}
	}
	*/
	//用乘法替代搜索，更高效
	Mat product; 
	//uchar temp;
	double temp;
	multiply(cam_DT, model_canny_img, product,1.0/255.0);
	cv::sort(product, product, CV_SORT_DESCENDING);

	for (int i = 0; i < product.size().height; i++) {
		
		for (int j = 0; j < product.size().width; j++) {
			//temp = product.at<uchar>(i, j);
			temp = product.at<float>(i, j);
			if (temp == 0) break;
			else dist.push_back(temp);
		}
	}
	sort(dist.begin(), dist.end());
	double sum = 0;
	for (int i = floor(k_u*dist.size()) - 1; i > floor(k_l*dist.size()); i--) {
		sum += dist[i];
	}
	std::cout << "max distance :" << dist[floor(k_u*dist.size()) - 1] << std::endl;
	return sum*dist[floor(k_u*dist.size()) - 1];
}
void MatchEdges::DT(Mat cam_img_, Mat &cam_DT_) const {
	bitwise_not(cam_img_, cam_img_);
	//distanceTransform(cam_img_, cam_DT_, CV_DIST_L1, 3, CV_8UC1);

	distanceTransform(cam_img_,cam_DT_, CV_DIST_L2, 3, CV_32FC1);
}
void MatchEdges::DT_L1(Mat cam_img_, Mat &cam_DT_) const {
	bitwise_not(cam_img_, cam_img_);
	distanceTransform(cam_img_, cam_DT_, CV_DIST_L1, 3, CV_8UC1);

	//distanceTransform(cam_img_, cam_DT_, CV_DIST_L2, 3, CV_32FC1);
}
double MatchEdges::DTmatchPyramid(double* var, int level, double k_l, double k_u) const {
	Mat model_canny_img;
	getModelImg<double>(var, model_canny_img);
	binaryZoomOut(model_canny_img, model_canny_img, 1/pow(2,level));
	return DTmatchHelp(cam_DT_pyramid[level], model_canny_img, k_l, k_u);
}

void MatchEdges::getCamCannyPoints(const Mat & cam_canny_img, vector<Point2i>& points)const {
	for (int i = 0; i < cam_canny_img.size().height; i++) {
		for (int j = 0; j < cam_canny_img.size().width; j++) {
			if (cam_canny_img.at<uchar>(i, j) > 0) points.push_back(Point2i(i, j));

		}
	}
}
void MatchEdges::binaryZoomOut(Mat input_img, Mat &output_img, double f)const {
	if (f == 1.0) {
		output_img = input_img;
		return;
	}
	resize(input_img, output_img, Size(0, 0), f, f, INTER_AREA);
	threshold(output_img, output_img, 20, 255, THRESH_BINARY);
}