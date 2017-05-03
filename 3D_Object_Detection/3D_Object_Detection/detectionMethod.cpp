
#include "detectionMethod.h"
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
	double deg_x = deg_x_l, deg_y = deg_y_l , deg_z = deg_z_l;
	SetEvent(readModelEvent);
	while (deg_x < deg_x_h) {
		while (deg_y < deg_y_h) {
			while (deg_z < deg_z_h) {
				WaitForSingleObject(sentModelEvent,INFINITE);
				cv::flip(readSrcImg, readSrcImg, 0);
				Canny(readSrcImg, model_canny_img_init, 500, 1000);
				model_ini_Canny_imgs.push_back(model_canny_img_init);
				//read the inital model imgs into a vector(buffer);



				deg_z = +precision_deg_z;
				SetEvent(readModelEvent);
			}

			deg_y = +precision_deg_y;
		}

		deg_x = +precision_deg_x;
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

	int i, cam_max_index, model_max_index; 
	double cam_max_area = 0; double model_max_area = 0; double temp_area;
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
	int num_steps_x = floor((deg_x_h - deg_x_l) / precision_deg_x) + 1;
	int num_steps_y = floor((deg_y_h - deg_y_l) / precision_deg_y) + 1;
	int num_steps_z = floor((deg_z_h - deg_z_l) / precision_deg_z) + 1;
	x_deg = floor(min_index /(num_steps_y*num_steps_z))*precision_deg_x + deg_x_l;
	y_deg = (min_index  % int(num_steps_y*num_steps_z))/ num_steps_y * precision_deg_y + deg_y_l;
	z_deg = (min_index % int(num_steps_z))*precision_deg_z + deg_z_l;
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
		std::cout << "deg_x_h, deg_y_h or deg_z_h need to be bigger" << endl;
	}
	if ((min_index + num_steps_y) % (num_steps_z*num_steps_y) < 2 * num_steps_y && (min_index + 1) % num_steps_z < 2) {
		std::cout << "deg_x_l,deg_y_l or deg_z_l need to be smaller" << endl;
	}
	//用二次曲线估计最佳点
	
	//估计的大致姿态与位置以及缩放系数
	deg_estimated[0] = curveEstimation(x_deg - precision_deg_x, scores[adj_index[0]], x_deg, scores[min_index], x_deg + precision_deg_x, scores[adj_index[1]]);
	deg_estimated[1] = curveEstimation(y_deg - precision_deg_y, scores[adj_index[2]], y_deg, scores[min_index], y_deg + precision_deg_y, scores[adj_index[3]]);
	deg_estimated[2] = curveEstimation(z_deg - precision_deg_z, scores[adj_index[4]], z_deg, scores[min_index], z_deg + precision_deg_z, scores[adj_index[5]]);
	centerPosEstimation(model_ini_Canny_imgs[min_index], cam_canny_img);

}

void PosDetection::shi_TomasiDetection() {
	//中精度定位：边缘图像角点匹配
	
	double output_best;
	//需先调用huCoarseDetection()
	PSO PSOer(&cam_canny_img, deg_estimated, pixel_pos_estimated, scale_ratio_estimated, 30, 6);
    //微调位置，姿态，z距离，缩放系数使partial Hausdorff distance(系数0.6)最小（或者模板匹配），采用粒子群算法
	PSOer.doPSO(var_best, output_best);
	cout << "Best Rotation is: x:" << deg_estimated[0] + var_best[0] << ", y:" << deg_estimated[1] + var_best[1] << ", z:" << deg_estimated[2] + var_best[2] << endl;
	cout << "Best Position is: x:" << pixel_pos_estimated[0] + var_best[3] << ", y:" << pixel_pos_estimated[1] + var_best[4] << endl;
	cout << "Best Scale Ratio is: " << scale_ratio_estimated + var_best[5] << endl;
}



//精定位方法1：边缘图像模板匹配，可缩放，平移。