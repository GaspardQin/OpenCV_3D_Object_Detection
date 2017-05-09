
#include "detectionMethod.h"
void PosDetection::vecmatwrite(const string& filename, const vector<Mat>& matrices)
{
	ofstream fs(filename, fstream::binary);

	for (size_t i = 0; i < matrices.size(); ++i)
	{
		const Mat& mat = matrices[i];

		// Header
		int type = mat.type();
		int channels = mat.channels();
		fs.write((char*)&mat.rows, sizeof(int));    // rows
		fs.write((char*)&mat.cols, sizeof(int));    // cols
		fs.write((char*)&type, sizeof(int));        // type
		fs.write((char*)&channels, sizeof(int));    // channels

													// Data
		if (mat.isContinuous())
		{
			fs.write(mat.ptr<char>(0), (mat.dataend - mat.datastart));
		}
		else
		{
			int rowsz = CV_ELEM_SIZE(type) * mat.cols;
			for (int r = 0; r < mat.rows; ++r)
			{
				fs.write(mat.ptr<char>(r), rowsz);
			}
		}
	}
}
vector<Mat> PosDetection::vecmatread(const string& filename)
{
	vector<Mat> matrices;
	ifstream fs(filename, fstream::binary);

	// Get length of file
	fs.seekg(0, fs.end);
	int length = fs.tellg();
	fs.seekg(0, fs.beg);

	while (fs.tellg() < length)
	{
		// Header
		int rows, cols, type, channels;
		fs.read((char*)&rows, sizeof(int));         // rows
		fs.read((char*)&cols, sizeof(int));         // cols
		fs.read((char*)&type, sizeof(int));         // type
		fs.read((char*)&channels, sizeof(int));     // channels

													// Data
		Mat mat(rows, cols, type);
		fs.read((char*)mat.data, CV_ELEM_SIZE(type) * rows * cols);

		matrices.push_back(mat);
	}
	return matrices;
}

template<typename T> int PosDetection::findMinIndex(std::vector<T>& src){
	int i = 0,min_index;
	T min = 10000; T temp;
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
template<typename T> int PosDetection::minElement(T* src, int size) {
	int i = 0;
	T min = 100000;
	for (i = 0; i < size; i++) {
		if (min > src[i]) {
			min = src[i];
		}
	}
	return min;

}
double PosDetection::curveEstimation(double x1, double y1, double x2, double y2, double x3, double y3) {
	//以三个点确定二次曲线最低点
	return (-((x1*x1 - x2*x2)*(y1 - y3) - (x1*x1 - x3*x3)*(y1 - y2) / (2 * (x2 - x3))));
}
void PosDetection::creatIniImgs() {
	rotate_degree_set[0] = deg_x_l;
	rotate_degree_set[1] = deg_y_l;
	rotate_degree_set[2] = deg_z_l;
	cv::Mat model_canny_img_init;
	//double deg_x = deg_x_l, deg_y = deg_y_l , deg_z = deg_z_l;
	SetEvent(readModelEvent);

	//refresh
	WaitForSingleObject(sentModelEvent,INFINITE);
	SetEvent(readModelEvent);
	while (rotate_degree_set[0] < deg_x_h) {
		rotate_degree_set[1] = deg_y_l;
		while (rotate_degree_set[1] < deg_y_h) {
			rotate_degree_set[2] = deg_z_l;
			while (rotate_degree_set[2] < deg_z_h) {
				WaitForSingleObject(sentModelEvent, INFINITE);
				cv::flip(readSrcImg, readSrcImg, 0);
				Canny(readSrcImg, model_canny_img_init, 80, 200);
				model_ini_Canny_imgs.push_back(model_canny_img_init.clone());
				//read the inital model imgs into a vector(buffer);

				std::cout << "Creating modelImg"<< model_ini_Canny_imgs.size()<<" degx,degy,degz" << rotate_degree_set[0] << "  " << rotate_degree_set[1] << "  " << rotate_degree_set[2] << endl;
				//imshow("test", model_canny_img_init);
				//waitKey();
				rotate_degree_set[2] += precision_deg_z;
				SetEvent(readModelEvent);
			}

			rotate_degree_set[1] += precision_deg_y;
		}

		rotate_degree_set[0] += precision_deg_x;
	}
	vecmatwrite("buffer.bin", model_ini_Canny_imgs);

}
void PosDetection::getInitialModelBuffer() {
	cout << "Initializing..." << endl;
	if (buffer_is_created == true) {
		model_ini_Canny_imgs = vecmatread("buffer.bin");
	}
	else creatIniImgs();
	
}
void PosDetection::initialization() {
	cam_src = imread("./model/sample.jpg", CV_8UC1);
	cam_src_color = imread("./model/sample.jpg");
	getInitialModelBuffer();
	Canny(cam_src, cam_canny_img, 50, 200);

	//粗定位
}
void PosDetection::debugShowContours(int canny_index,vector<vector<Point>> *cam_contours,int cam_index, vector<vector<Point>> *model_contours,int model_index) {
	Mat back_ground = cam_src_color.clone();
	back_ground.setTo(cv::Scalar(255, 255, 255));
	drawContours(back_ground, *cam_contours, cam_index,Scalar(255,0,0));
	drawContours(back_ground, *model_contours, model_index, Scalar(0, 0, 255));
	imshow("debugShowContours", back_ground);
	imshow("debugShowModelCanny", model_ini_Canny_imgs[canny_index]);
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
	mo = moments(cam_contours[cam_max_index],true);
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
	debugShowContours(index, &cam_contours, cam_max_index, &model_contours, model_max_index);
	
	
	match_score = matchShapes(cam_contours[cam_max_index], model_contours[model_max_index], CV_CONTOURS_MATCH_I3,0);
	//分数越低，越匹配。
	cout << "match_score is " << match_score << endl;

	waitKey(1000);
	return match_score;
}
void PosDetection::huCoarseDetection() {
	//不会用于本次应用，但保留该方法
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
		adj_index[4] = min_index + 1;
		adj_index[5] = min_index - 1;
	if (minElement(adj_index, 6) < 0) {
			std::cout << "deg_x_l, deg_y_l or deg_z_l need to be smaller" << endl;
	}
	if(maxElement(adj_index,6) >= model_ini_Canny_imgs.size()) {
		std::cout << "deg_x_h, deg_y_h or deg_z_h need to be bigger" << endl;
	}
	if ((min_index + num_steps_y) % (num_steps_z*num_steps_y) < 2 * num_steps_y && (min_index + 1) % num_steps_z < 2) {
		std::cout << "deg_x_l,deg_y_l or deg_z_l need to be smaller" << endl;
	}
	//用二次曲线估计最佳点
	
	//估计的大致姿态与位置以及缩放系数
	rotate_degree_estimated[0] = curveEstimation(x_deg - precision_deg_x, scores[adj_index[0]], x_deg, scores[min_index], x_deg + precision_deg_x, scores[adj_index[1]]);
	rotate_degree_estimated[1] = curveEstimation(y_deg - precision_deg_y, scores[adj_index[2]], y_deg, scores[min_index], y_deg + precision_deg_y, scores[adj_index[3]]);
	rotate_degree_estimated[2] = curveEstimation(z_deg - precision_deg_z, scores[adj_index[4]], z_deg, scores[min_index], z_deg + precision_deg_z, scores[adj_index[5]]);
	centerPosEstimation(model_ini_Canny_imgs[min_index], cam_canny_img);

}

void PosDetection::shi_TomasiDetection(double* output_best) {
	//应输入大小为6的数组以接受最佳匹配位置及姿态
	//精精度定位：边缘图像角点匹配
	
	
	//需先调用至少一种粗定位函数确保匹配模板和待匹配图像像素位置有重合
	MatchSolver matchSolver;
	matchSolver.setIniVar(pos_estimated[0], pos_estimated[1], pos_estimated[2], rotate_degree_estimated[0], rotate_degree_estimated[1], rotate_degree_estimated[2]);
	matchSolver.solve(&cam_canny_img, output_best);
		
	
    //微调位置，姿态使partial Hausdorff distance(系数0.6)最小（或者模板匹配），采用LM非线性最优化算法
	cout << "Best Position is: x:" << output_best[0] << ", y:" << output_best[1] << ", z:"<< output_best[2] << endl;
	cout << "Best Rotation is: x:" << output_best[3] << ", y:" << output_best[4] << ", z:" << output_best[5] << endl;

}



//精定位方法1：边缘图像模板匹配，可缩放，平移。