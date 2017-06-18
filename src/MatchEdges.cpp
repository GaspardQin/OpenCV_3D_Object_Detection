
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
	int column_left_pixel = round(WINDOW_WIDTH / 2 + x / (CCD_WIDTH / 2 / FOCAL_DISTANCE*(-z))*WINDOW_WIDTH - ROI_WIDTH / 2);//z为负数
	int row_top_pixel = round(WINDOW_HEIGHT / 2 - y / (CCD_WIDTH / 2 / FOCAL_DISTANCE*(-z) / WINDOW_WIDTH*WINDOW_HEIGHT)*WINDOW_HEIGHT - ROI_HEIGHT / 2);
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


void MatchEdges::getModelImgUchar(const int* var) const {
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
	SetEvent(nextModelEvent);

	WaitForSingleObject(sentModelEvent, INFINITE);
	//cvtColor(readSrcImg, model_canny_img, CV_RGB2GRAY);
	//cv::flip(readSrcImg, readSrcImg, 0);
	//Mat model_canny_img_pre;
	//Canny(readSrcImg, model_canny_img, 50, 200);
}
void MatchEdges::getModelImgUchar(const double* var) const {
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
	SetEvent(nextModelEvent);

	WaitForSingleObject(sentModelEvent, INFINITE);
	//cvtColor(readSrcImg, model_canny_img, CV_RGB2GRAY);
	//cv::flip(readSrcImg, readSrcImg, 0);
	//Mat model_canny_img_pre;
	//Canny(readSrcImg, model_canny_img, 50, 200);
	//lock.~lock_guard();
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

double MatchEdges::MatchOnline_modelCannycamDT(const int* var, double k_l, double k_u) const {

	int index = discrete_info.getIndex(var);
	if (cache_match[index] >= 0) { 
		boost::shared_lock<boost::shared_mutex> lock(cv_cache_mutex);//读取操作是采用读取锁（可多线程同时读取）
		return cache_match[index];
	}


	vector<Point2i> point_vec;
	vector<double> dist;
	double temp;

	getModelImgUchar(var);
	cv::findNonZero(readSrcImg, point_vec);
	
	ResetEvent(readImgEvent);
	SetEvent(readImgEvent);


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
	boost::lock_guard<boost::shared_mutex> lock(cv_cache_mutex); //保证每个时刻只有一个线程能写入cache_match;
	cache_match[index] = sum; //存入cache；

	return sum;// *dist[floor(k_u*dist.size()) - 1];

}
double MatchEdges::MatchOnline_modelDTcamCanny(const int* var, double k_l, double k_u) const {
	//需要重写

	int index = discrete_info.getIndex(var);
	if (cache_match[index] >= 0) {
		boost::shared_lock<boost::shared_mutex> lock(cv_cache_mutex);//读取操作是采用读取锁（可多线程同时读取）
		return cache_match[index];
	}

	Mat model_DT;
	vector<double> dist;
	dist.reserve(cam_canny_points.size());
	double temp;
	Mat bit_not_src;
	getModelImgUchar(var);
	bitwise_not(readSrcImg, bit_not_src);

	ResetEvent(readImgEvent);
	SetEvent(readImgEvent);

	distanceTransform(bit_not_src, model_DT, CV_DIST_L2, 5, CV_32FC1);

	float* row_modelDT_ptrs[int(WINDOW_HEIGHT)];
 	for (int i = 0; i < model_DT.rows; i++) {
		row_modelDT_ptrs[i] = model_DT.ptr<float>(i);
	}

	for (std::vector<Point2i>::iterator iter = cam_canny_points.begin(); iter < cam_canny_points.end(); iter++) {
		temp = row_modelDT_ptrs[iter->y][iter->x];//findNonZero得到的Point x,y与Mat中的坐标相反
		//if (temp > 0) 
		dist.push_back(temp);
	}

	sort(dist.begin(), dist.end());

	double sum = 0;
	if (dist.size() == 0) dist.push_back(0);
	else {
		for (int i = floor(k_u*dist.size()); i > floor(k_l*dist.size()); i--) {
			//sum += dist[i];
			sum += dist[i] * dist[i];
		}
	}
	sum = sqrt(sum / double((floor(k_u*dist.size()) - floor(k_l*dist.size()))))/3.0;
	std::cout << "max distance :" << dist[floor(k_u*dist.size())] << std::endl;
	boost::lock_guard<boost::shared_mutex> lock(cv_cache_mutex); //保证每个时刻只有一个线程能写入cache_match;
	cache_match[index] = sum; //存入cache；

	return sum;// *dist[floor(k_u*dist.size()) - 1];


}
double MatchEdges::MatchOnline_modelDTcamCannyROI(const int* var, double k_l, double k_u) const {
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
	int index = discrete_info.getIndex(var);
	if (cache_match[index] >= 0) {
		boost::shared_lock<boost::shared_mutex> lock(cv_cache_mutex);//读取操作是采用读取锁（可多线程同时读取）
		return cache_match[index];
	}

	Mat model_DT;
	vector<double> dist;
	double temp;
	Mat bit_not_src;
	getModelImgUchar(var);
	bitwise_not(readSrcImgROI, bit_not_src);

	ResetEvent(readImgEvent);
	SetEvent(readImgEvent);

	distanceTransform(bit_not_src, model_DT, CV_DIST_L2, 5, CV_8UC1);

	uchar* row_modelDT_ptrs[int(WINDOW_HEIGHT)];
	for (int i = 0; i < model_DT.rows; i++) {
		row_modelDT_ptrs[i] = model_DT.ptr<uchar>(i);
	}
	int rect_pixel[4];//column_left_pixel,row_top_pixel,,column_right_pixel,row_bottom_pixel
	getROIrect(double(var[0]), double(var[1]), double(var[2]), rect_pixel);

	//Mat temp_debug;
	//getROI(readSrcImg, temp_debug, double(var[0]), double(var[1]), double(var[2]));
	for (std::vector<Point2i>::iterator iter = cam_canny_points.begin(); iter < cam_canny_points.end(); iter++) {
		int row_temp = iter->y;
	    int col_temp = iter->x;

		//iter->y row, iter->x col
		if (col_temp < rect_pixel[0]) col_temp = rect_pixel[0];
		else if (col_temp >= rect_pixel[2]) col_temp = rect_pixel[2] -1;

		if (row_temp< rect_pixel[1]) row_temp = rect_pixel[1];
		else if (row_temp >= rect_pixel[3]) row_temp = rect_pixel[3] -1;
		temp = row_modelDT_ptrs[row_temp - rect_pixel[1]][col_temp - rect_pixel[0]];//findNonZero得到的Point x,y与Mat中的坐标相反
		if (temp > 0) dist.push_back(temp);
	}
	sort(dist.begin(), dist.end());

	double sum = 0;
	if (dist.size() == 0) dist.push_back(0);
	else {
		for (int i = floor(k_u*dist.size()); i > floor(k_l*dist.size()); i--) {
			sum += dist[i]* dist[i];
		}
	}
	sum = sqrt(sum / double((floor(k_u*dist.size()) - floor(k_l*dist.size())))) / 3.0;
	std::cout << "max distance :" << dist[floor(k_u*dist.size())] << std::endl;
	boost::lock_guard<boost::shared_mutex> lock(cv_cache_mutex); //保证每个时刻只有一个线程能写入cache_match;
	cache_match[index] = sum; //存入cache；

	return sum;// *dist[floor(k_u*dist.size()) - 1];

}
double MatchEdges::MatchOnline_modelDTcamCannyROI_continuous(const double* var, double k_l, double k_u) const {
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
	Mat model_DT;
	vector<double> dist;
	dist.reserve(cam_canny_points.size());
	double temp;
	Mat bit_not_src;
	getModelImgUchar(var);
	bitwise_not(readSrcImgROI, bit_not_src);

	ResetEvent(readImgEvent);
	SetEvent(readImgEvent);

	distanceTransform(bit_not_src, model_DT, CV_DIST_L1, 3, CV_8UC1);

	uchar* row_modelDT_ptrs[int(ROI_HEIGHT)];
	for (int i = 0; i < model_DT.rows; i++) {
		row_modelDT_ptrs[i] = model_DT.ptr<uchar>(i);
	}
	int rect_pixel[4];//column_left_pixel,row_top_pixel,,column_right_pixel,row_bottom_pixel
	getROIrect(double(var[0]), double(var[1]), double(var[2]), rect_pixel);

	//Mat temp_debug;
	//getROI(readSrcImg, temp_debug, double(var[0]), double(var[1]), double(var[2]));
	for (std::vector<Point2i>::iterator iter = cam_canny_points.begin(); iter < cam_canny_points.end(); iter++) {
		int row_temp = iter->y;
		int col_temp = iter->x;

		//iter->y row, iter->x col
		if (col_temp < rect_pixel[0]) col_temp = rect_pixel[0];
		else if (col_temp >= rect_pixel[2]) col_temp = rect_pixel[2]-1;

		if (row_temp< rect_pixel[1]) row_temp = rect_pixel[1];
		else if (row_temp >= rect_pixel[3]) row_temp = rect_pixel[3]-1;
		temp = row_modelDT_ptrs[row_temp - rect_pixel[1]][col_temp - rect_pixel[0]];//findNonZero得到的Point x,y与Mat中的坐标相反
		//if (temp > 0) 
		dist.push_back(temp);
	}
	sort(dist.begin(), dist.end());

	double sum = 0;
	if (dist.size() == 0) dist.push_back(0);
	else {
		for (int i = floor(k_u*dist.size()); i > floor(k_l*dist.size()); i--) {
			sum += dist[i]* dist[i];
		}
	}
	sum = sqrt(sum / double((floor(k_u*dist.size()) - floor(k_l*dist.size())))) / 3.0;
	std::cout << "max distance :" << dist[floor(k_u*dist.size())] << std::endl;
	return sum;// *dist[floor(k_u*dist.size()) - 1];

}
double MatchEdges::MatchOnline_modelDTcamCanny_continuous(const double* var, double k_l, double k_u) const {
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
	Mat model_DT;
	vector<double> dist;
	dist.reserve(cam_canny_points.size());
	double temp;
	Mat bit_not_src;
	getModelImgUchar(var);
	bitwise_not(readSrcImg, bit_not_src);

	ResetEvent(readImgEvent);
	SetEvent(readImgEvent);

	distanceTransform(bit_not_src, model_DT, CV_DIST_L2, 3, CV_32FC1);

	float* row_modelDT_ptrs[int(WINDOW_HEIGHT)];
	for (int i = 0; i < model_DT.rows; i++) {
		row_modelDT_ptrs[i] = model_DT.ptr<float>(i);
	}
	for (std::vector<Point2i>::iterator iter = cam_canny_points.begin(); iter < cam_canny_points.end(); iter++) {
		temp = row_modelDT_ptrs[iter->y][iter->x];//findNonZero得到的Point x,y与Mat中的坐标相反
																//if (temp > 0) 
		dist.push_back(temp);
	}
	sort(dist.begin(), dist.end());

	double sum = 0;
	if (dist.size() == 0) dist.push_back(0);
	else {
		for (int i = floor(k_u*dist.size()); i > floor(k_l*dist.size()); i--) {
			sum += dist[i] * dist[i];
		}
	}
	sum = sqrt(sum / double((floor(k_u*dist.size()) - floor(k_l*dist.size())))) / 3.0;
	std::cout << "max distance :" << dist[floor(k_u*dist.size())] << std::endl;
	return sum;// *dist[floor(k_u*dist.size()) - 1];

}
double MatchEdges::MatchOffline_modelCannycamDT(const int* var, double k_l, double k_u) const {
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
	boost::lock_guard<boost::shared_mutex> lock(cv_cache_mutex); //保证每个时刻只有一个线程能写入cache_match;
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