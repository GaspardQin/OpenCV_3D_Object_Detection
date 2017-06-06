
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
	cvtColor(readSrcImg, model_canny_img, CV_RGB2GRAY);
	//cv::flip(readSrcImg, readSrcImg, 0);
	//Mat model_canny_img_pre;
	//Canny(readSrcImg, model_canny_img, 50, 200);
}

double MatchEdges::DTmatchHelp(Mat cam_DT, Mat model_canny_img, double k_l, double k_u) const {
	//DistanceTransform Match
	vector<double> dist;

	//Mat product; 

	double temp;
	//multiply(cam_DT, model_canny_img, product,1.0/255.0);
	//cv::sort(product, product, CV_SORT_DESCENDING);

	//for (int i = 0; i < product.size().height; i++) {
		
	//	for (int j = 0; j < product.size().width; j++) {
			//temp = product.at<uchar>(i, j);
	//		temp = product.at<float>(i, j);
//			if (temp == 0) break;
//			else dist.push_back(temp);
	//	}
//	}


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

double MatchEdges::DTmatchOnlinePyramid(int* var, int level, double k_l, double k_u) const {

	int index = getIndex(var);
	if (cache_match[index] >= 0) { //读取操作是线程安全的
		return cache_match[index];
	}


	Mat model_canny_img;
	vector<Point2i> point_vec;
	vector<double> dist;
	double temp;

	getModelImgUchar(var, model_canny_img);
	if (level > 0) {
		binaryZoomOut(model_canny_img, model_canny_img, 1 / pow(2, level));
	}
	cv::findNonZero(model_canny_img, point_vec);

	for (std::vector<Point2i>::iterator iter = point_vec.begin(); iter < point_vec.end(); iter++) {
		temp = row_camDT_ptrs[iter->y][iter->x];//findNonZero得到的Point x,y与Mat中的坐标相反
		if (temp > 0) dist.push_back(temp);
	}

	sort(dist.begin(), dist.end());

	double sum = 0;
	if (dist.size() == 0) dist.push_back(0);
	else {
		for (int i = floor(k_u*dist.size()); i > floor(k_l*dist.size()); i--) {
			sum += dist[i];
		}
	}

	std::cout << "max distance :" << dist[floor(k_u*dist.size())] << std::endl;
	boost::lock_guard<boost::mutex> lock(cv_cache_mutex); //保证每个时刻只有一个线程能写入cache_match;
	cache_match[index] = sum; //存入cache；

	return sum;// *dist[floor(k_u*dist.size()) - 1];

}


void MatchEdges::binaryZoomOut(Mat input_img, Mat &output_img, double f)const {
	if (f == 1.0) {
		output_img = input_img;
		return;
	}
	resize(input_img, output_img, Size(0, 0), f, f, INTER_AREA);
	threshold(output_img, output_img, 20, 255, THRESH_BINARY);
}