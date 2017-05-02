//get initial Model Buffer function in this files
#include "thread_variables.h"

using namespace std;
using namespace cv;
class PosDetection {
public:
	std::vector<cv::Mat> model_ini_Canny_imgs;
	double deg_estimated[3]; double pixel_pos_estimated[2]; double scale_ratio_estimated;
	cv::Mat cam_canny_img, cam_src;
	int min_index;
	template<typename T> int findMinIndex(std::vector<T>& src);
	template<typename T> int findMaxIndex(std::vector<T>& src);
	template<typename T> int maxElement(T* src, int size);
	void getInitialModelBuffer();
	void initialization();
	double huMatchFromCannyImg(int index);
	void PosDetection::centerPosEstimation(Mat &model_img, Mat &cam_img);
	void huCoarseDetection();
	double curveEstimation(double x1,double y1, double x2,double y2, double x3, double y3);
	void shi_TomasiMatchFromCannyImgs();
	PosDetection(double camera_z_set_input, double precision_x_input, double precision_y_input, double precision_z_input, double x_l_input, double x_h_input, double y_l_input, double y_h_input, double z_l_input, double z_h_input) {
		camera_z_set_input = camera_z_set;
		precision_x_input = precision_x;
		precision_y_input = precision_y;
		precision_z_input = precision_z; 
		x_l_input = x_l;
		x_h_input = x_h;
		y_l_input = y_l;
		y_h_input = y_h;
		z_l_input = z_l;
		z_h_input = z_h;
	}
private:
	double camera_z_set;
	double precision_x;
	double precision_y;
	double precision_z;
	double x_l;
	double x_h;
	double y_l;
	double y_h;
	double z_l;
	double z_h;

};
template<typename T> int PosDetection::findMinIndex(std::vector<T>& src){
	int i = 0,min_index;
	T min = 0; T temp;
	for (i = 0; i < src.size(); i++) {
		if (min > src[i]) {
			min = src[i];
			min_index = i;
		}
	}
	return min_index;

}
template<typename T> int PosDetection::findMaxIndex(std::vector<T>& src) {
	int i = 0, max_index;
	T max = 0;
	for (i = 0; i < src.size(); i++) {
		if (max < src[i]) {
			max = src[i];
			max_index = i;
		}
	}
	return max_index;

}
template<typename T> int PosDetection::maxElement(T* src,int size) {
	int i = 0;
	T max = 0;
	for (i = 0; i < size ; i++) {
		if (max < src[i]) {
			max = src[i];
		}
	}
	return max;

}
double PosDetection::curveEstimation(double x1, double y1, double x2, double y2, double x3, double y3) {
	return (-((x1*x1 - x2*x2)*(y1 - y3) - (x1*x1 - x3*x3)*(y1 - y2) / (2 * (x2 - x3))));
}
void PosDetection::getInitialModelBuffer() {
	cv::Mat model_canny_img_init;
	WaitForSingleObject(sentModelEvent, INFINITE);
	rotate_degree[0] = 0;
	rotate_degree[1] = 0;
	rotate_degree[2] = 0;
	SetEvent(readModelEvent);
	camera_z = camera_z_set;
	double deg_x = x_l, deg_y = y_l , deg_z = z_l;
	SetEvent(readModelEvent);
	while (deg_x < x_h) {
		while (deg_y < y_h) {
			while (deg_z < z_h) {
				WaitForSingleObject(sentModelEvent,INFINITE);
				cv::flip(readSrcImg, readSrcImg, 0);
				Canny(readSrcImg, model_canny_img_init, 500, 1000);
				model_ini_Canny_imgs.push_back(model_canny_img_init);
				//read the inital model imgs into a vector(buffer);



				deg_z = +precision_z;
				SetEvent(readModelEvent);
			}

			deg_y = +precision_y;
		}

		deg_x = +precision_x;
	}
	
	
}
void PosDetection::initialization() {
	getInitialModelBuffer();
	Canny(cam_src, cam_canny_img, 500, 1000);

	//粗定位
}
void PosDetection::centerPosEstimation(Mat &model_img,Mat &cam_img){
	vector<Vec4i> hierarchy;
	vector<vector<Point> > cam_contours, model_contours;
	findContours(cam_img, cam_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	findContours(model_img, model_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	int i, cam_max_index, model_max_index; double cam_max_area = 0, double model_max_area = 0;double temp_area;
	for (i = 0; i < cam_contours.size(); i++) {
		temp_area = contourArea(cam_contours[i]);
		if (cam_max_area < temp_area) {
			cam_max_index = i;
			cam_max_area = temp_area;
		}
	}
	for (i = 0; i < model_contours.size(); i++) {
		temp_area = contourArea(model_contours[i]);
		if (model_max_area < temp_area) {
			model_max_index = i;
			model_max_area = temp_area;
		}
	}
	Moments mo; double pixel_pos_estimated[2];
	mo = moments(cam_contours[i],true);
	pixel_pos_estimated[0] = mo.m10 / mo.m00;
	pixel_pos_estimated[1] = mo.m01 / mo.m00;
	double scale_ratio_estimated = model_max_area / cam_max_area;
}
double PosDetection::huMatchFromCannyImg(int index) {
	Mat model_canny_img = model_ini_Canny_imgs[index];
	double match_score;
	vector<vector<Point> > cam_contours, model_contours;
	vector<Vec4i> hierarchy;
	int i, cam_max_index,model_max_index;
	double cam_max_area = 0.0,model_max_area = 0,temp_area;
	findContours(cam_canny_img, cam_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	findContours(model_canny_img, model_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	for (i = 0; i < cam_contours.size(); i++) {
		temp_area = contourArea(cam_contours[i]);
		if (cam_max_area < temp_area) {
			cam_max_index = i;
			cam_max_area = temp_area;
		}
	}
	for (i = 0; i < model_contours.size(); i++) {
		temp_area = contourArea(model_contours[i]);
		if (model_max_area < temp_area) {
			model_max_index = i;
			model_max_area = temp_area;
		}
	}
	
	match_score = matchShapes(cam_contours[cam_max_index], model_contours[model_max_index], CV_CONTOURS_MATCH_I3,0);
	//分数越低，越匹配。
	return match_score;
}
void PosDetection::huCoarseDetection() {
	//粗定位方法1：外轮廓HU姿态确定，仅确定姿态
	//需注意HU矩有旋转不变矩！旋转角度不可太大，最好不能在该范围内有对称图案！
	int i;
	vector<double> scores;
	for (i = 0; i < model_ini_Canny_imgs.size(); i++) {
		scores.push_back(huMatchFromCannyImg(i));
	}
	min_index = findMinIndex(scores);
	//寻找相邻计算点
	double x_deg, y_deg, z_deg;
	int num_steps_x = floor((x_h - x_l) / precision_x) + 1;
	int num_steps_y = floor((y_h - y_l) / precision_y) + 1;
	int num_steps_z = floor((z_h - z_l) / precision_z) + 1;
	x_deg = floor(min_index /(num_steps_y*num_steps_z))*precision_x + x_l;
	y_deg = (min_index  % int(num_steps_y*num_steps_z))/ num_steps_y * precision_y + y_l;
	z_deg = (min_index % int(num_steps_z))*precision_z + z_l;
	int adj_index[6]; //x,y,z分别相邻的6点
	//x
		adj_index[0] = min_index - num_steps_y*num_steps_z;
		adj_index[1] = min_index - num_steps_y*num_steps_z;
	//y
		adj_index[2] = min_index - num_steps_z;
		adj_index[3] = min_index + num_steps_z;
	//z
		adj_index[3] = min_index + 1;
		adj_index[3] = min_index - 1;
	if(maxElement(adj_index,6) >= model_ini_Canny_imgs.size()) {
		std::cout << "x_h, y_h or z_h need to be bigger" << endl;
	}
	if ((min_index + num_steps_y) % (num_steps_z*num_steps_y) < 2 * num_steps_y && (min_index + 1) % num_steps_z < 2) {
		std::cout << "x_l,y_l or z_l need to be smaller" << endl;
	}
	//用二次曲线估计最佳点
	
	//估计的大致姿态与位置以及缩放系数
	deg_estimated[0] = curveEstimation(x_deg - precision_x, scores[adj_index[0]], x_deg, scores[min_index], x_deg + precision_x, scores[adj_index[1]]);
	deg_estimated[1] = curveEstimation(y_deg - precision_y, scores[adj_index[2]], y_deg, scores[min_index], y_deg + precision_y, scores[adj_index[3]]);
	deg_estimated[2] = curveEstimation(z_deg - precision_z, scores[adj_index[4]], z_deg, scores[min_index], z_deg + precision_z, scores[adj_index[5]]);
	centerPosEstimation(model_ini_Canny_imgs[min_index], cam_canny_img);

}

void PosDetection::shi_TomasiMatchFromCannyImgs() {
	//粗定位方法2：边缘图像角点匹配

	//需先调用huCoarseDetection()
	int maxCorners = 30;    //角点个数的最大值  
	/// Shi-Tomasi的参数设置  
	vector<Point2f> cam_corners,model_corners;
	double qualityLevel = 0.01;
	double minDistance = 10;
	int blockSize = 3;
	bool useHarrisDetector = false; //不使用Harris检测算法  
	double k = 0.04;

	/// 深度拷贝原图像用于绘制角点  
	//Mat srcCopy = src.clone();
	/// 应用角点检测算法  
	goodFeaturesToTrack(cam_canny_img, cam_corners, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k); //未选择感兴趣区域  
	goodFeaturesToTrack(model_ini_Canny_imgs[min_index], model_corners, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k); //未选择感兴趣区域  
	
	
	cv::Ptr <cv::HausdorffDistanceExtractor> hausdorff_ptr = cv::createHausdorffDistanceExtractor();
	hausdorff_ptr->setRankProportion(0.6);
	float distance = hausdorff_ptr->computeDistance(cam_corners, model_corners);
    //微调位置，姿态，z距离，缩放系数使partial Hausdorff distance(系数0.6)最小（或者模板匹配），采用粒子群算法
}

class MatchPSO {
public:
	Mat* cam_img;
	double* deg_estimated_before;
	double* pixel_pos_estimated_before;
	double scale_ratio_estimated_before;
	
	//角点参数设置
	// Shi-Tomasi的参数设置  
	vector<Point2f> cam_corners_PSO;
	int maxCorners = 30;    //角点个数的最大值  
	double qualityLevel = 0.01;
	double minDistance = 10;
	int blockSize = 3;
	bool useHarrisDetector = false; //不使用Harris检测算法  
	double k = 0.04;

	cv::Ptr <cv::HausdorffDistanceExtractor> hausdorff_ptr = cv::createHausdorffDistanceExtractor();
	

	int dim = 6;//总共六维
	MatchPSO(Mat *cam_img_input, double* deg_input, double *pixel_pos_input, double scale_ratio_estimated_before_input) {
		cam_img = cam_img_input;
		deg_estimated_before = deg_input;//3维
		pixel_pos_estimated_before = pixel_pos_input;//2维
		scale_ratio_estimated_before = scale_ratio_estimated_before_input;//1维
		hausdorff_ptr->setRankProportion(0.6);
	};
	double hausdorffCost(double *var);//x是六维向量
	void camCornerDect();
	void getModelImg(double *var, Mat& model_canny_img);
};
void MatchPSO::camCornerDect() {
	goodFeaturesToTrack(*cam_img, cam_corners_PSO, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k); //未选择感兴趣区域  
}

void MatchPSO::getModelImg(double *var, Mat& model_canny_img) {
	rotate_degree[0] = deg_estimated_before[0] + var[0];
	rotate_degree[1] = deg_estimated_before[1] + var[1];
	rotate_degree[2] = deg_estimated_before[2] + var[2];
	SetEvent(readModelEvent);
	WaitForSingleObject(sentModelEvent,INFINITE);
	cv::flip(readSrcImg, readSrcImg, 0);
	Mat model_canny_img_pre;
	Canny(readSrcImg, model_canny_img_pre, 500, 1000);
	Mat warp_mat(2, 3, CV_32FC1);
	warp_mat.at<double>(0,0) = scale_ratio_estimated_before+ var[5];
	warp_mat.at<double>(0,1) = 0;
	warp_mat.at<double>(1, 0) = scale_ratio_estimated_before + var[5];
	warp_mat.at<double>(1, 1) = 0;
	warp_mat.at<double>(0,2) = pixel_pos_estimated_before[0] + var[3];
	warp_mat.at<double>(1,2) = pixel_pos_estimated_before[1] + var[4];
	warpAffine(model_canny_img_pre, model_canny_img, warp_mat, model_canny_img_pre.size());
}

double MatchPSO::hausdorffCost(double *var) {
	//粒子群cost函数
	Mat model_canny_img;
	vector<Point2f> model_corners_PSO;
	getModelImg(var, model_canny_img);
	goodFeaturesToTrack(model_canny_img, model_corners_PSO, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k); //未选择感兴趣区域  
	
	return (hausdorff_ptr->computeDistance(cam_corners_PSO, model_corners_PSO));
	//微调位置，姿态，z距离，缩放系数使partial Hausdorff distance(系数0.6)最小（或者模板匹配），采用粒子群算法
}



//精定位方法1：边缘图像模板匹配，可缩放，平移。