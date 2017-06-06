
#include "detectionMethodWithoutBuffer.h"

int iteral_count = 0;
template<typename T> int DetectionMethod::findMinIndex(std::vector<T>& src){
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
template<typename T> int DetectionMethod::findMaxIndex(std::vector<T>& src) {
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
template<typename T> int DetectionMethod::maxElement(T* src,int size) {
	int i = 0;
	T max = 0;
	for (i = 0; i < size ; i++) {
		if (max < src[i]) {
			max = src[i];
		}
	}
	return max;

}
template<typename T> int DetectionMethod::minElement(T* src, int size) {
	int i = 0;
	T min = 100000;
	for (i = 0; i < size; i++) {
		if (min > src[i]) {
			min = src[i];
		}
	}
	return min;

}
void DetectionMethod::arrayVecOfPointsWrite(const string& filename, const vector<Point2i>* points_vector_array, const int array_size)
{
	ofstream fs(filename, fstream::binary);
	const size_t point2i_size =  sizeof(Point2i);
	vector<int> vectors_size;
	vectors_size.reserve(array_size);
	for (size_t i = 0; i < array_size; ++i) {
		vectors_size.push_back(points_vector_array[i].size());
	}
	fs.write((char*)&vectors_size[0], sizeof(int)*array_size);

	for (size_t i = 0; i < array_size; ++i)
	{
		//const vector<Point2i>& points_vector = points_vector_array[i];

		//for (size_t j = 0; j < vector_size; j++) {
			fs.write((char*)&points_vector_array[i][0], points_vector_array[i].size()*point2i_size);
	//	}
		cout.setf(ios::fixed);
		std::cout << "Writing ... " << double(i) / double(array_size) * 100 << "%" << endl;
	}
}
void DetectionMethod::arrayVecOfPointsRead(const string& filename, int array_size)
{
	vector<Point2i> *p = new vector<Point2i>[array_size];
	boost::shared_array<vector<Point2i>> model_canny_points_(p);
	ifstream fs(filename, fstream::binary);
	if (!fs)
	{
		while (1) {
			std::cout << "Error: " << filename << " not found!" << std::endl;
		}
	}
	const size_t point2i_size = sizeof(Point2i);
	// Get length of file
	fs.seekg(0, fs.end);
	//int length = fs.tellg();//tellg()应对2G以上文件时会出现bug
	__int64 length = *(__int64*)(((char*)&(fs.tellg())) + 8);
	fs.seekg(0, fs.beg);

	int count = 0;
	vector<int> vectors_size(array_size);

	fs.read(reinterpret_cast<char*>(&vectors_size[0]), array_size*sizeof(vectors_size[0]));
	__int64 debug0 = *(__int64*)(((char*)&(fs.tellg())) + 8);
	while (*(__int64*)(((char*)&(fs.tellg())) + 8) < length)
	{
		// Data
		vector<Point2i> vec_point_temp(vectors_size[count]);
		fs.read((char*)&vec_point_temp[0], point2i_size*vectors_size[count]);
		model_canny_points_[count] = vec_point_temp;
		vec_point_temp.~vector();
		count++;
		cout.setf(ios::fixed);
		std::cout << "Reading... " << double(count)/ double(array_size)*100<<" %" << endl;
		__int64 debug1 = *(__int64*)(((char*)&(fs.tellg())) + 8);
	}
	if (count != array_size) {
		std::cout << "Error: array_size is not correct! " << std::endl;
	}
	
	model_canny_points = model_canny_points_;
}

void DetectionMethod::arrayMatWrite(const string& filename, const Mat* matrices, const int array_size)
{
	ofstream fs(filename, fstream::binary);

	for (size_t i = 0; i < array_size; ++i)
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
		cout.setf(ios::fixed);
		std::cout << "Writing ... " << double(i) / double(array_size) *100<<"%"<< endl;
	}
}
void DetectionMethod::arrayMatRead(const string& filename, int array_size)
{
	Mat *p = new Mat[array_size];
	boost::shared_array<Mat> model_DT_mats(p);
	ifstream fs(filename, fstream::binary);
	if (!fs)
	{
		while (1) {
			std::cout << "Error: " << filename << " not found!" << std::endl;
		}
	}
	// Get length of file
	fs.seekg(0, fs.end);
	int length = fs.tellg();
	fs.seekg(0, fs.beg);

	int count=0;
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

		model_DT_mats[count] = mat;
		count++;
	}
	if (count != array_size) {
		std::cout << "Error: array_size is not correct! " << std::endl;
	}
	model_DT_imgs = model_DT_mats;
}
void DetectionMethod::setBufferInitValue(int x_init, int y_init, int z_init, int deg_x, int deg_y, int deg_z) {
	init_buffer_var[0] = x_init;
	init_buffer_var[1] = y_init;
	init_buffer_var[2] = z_init;
	init_buffer_var[3] = deg_x;
	init_buffer_var[4] = deg_y;
	init_buffer_var[5] = deg_z;
}
void DetectionMethod::setBufferPrecision(int x_precision,int y_precision, int z_precision, int x_deg_precision, int y_deg_precision, int z_deg_precision ) {
	init_buffer_precision[0] = x_precision;
	init_buffer_precision[1] = y_precision;
	init_buffer_precision[2] = z_precision;
	init_buffer_precision[3] = x_deg_precision;
	init_buffer_precision[4] = y_deg_precision;
	init_buffer_precision[5] = z_deg_precision;

}

void DetectionMethod::setBufferBoundary(int delta_x, int delta_y, int delta_z, int delta_x_deg, int delta_y_deg, int delta_z_deg) {
	//边界与中心值差值，上下边界关于中心值对称
	init_buffer_delta[0] = delta_x;
	init_buffer_delta[1] = delta_y;
	init_buffer_delta[2] = delta_z;
	init_buffer_delta[3] = delta_x_deg;
	init_buffer_delta[4] = delta_y_deg;
	init_buffer_delta[5] = delta_z_deg;

	init_buffer_num[0] = 2 * floor(delta_x / init_buffer_precision[0]) + 1;
	init_buffer_num[1] = 2 * floor(delta_y / init_buffer_precision[1]) + 1;
	init_buffer_num[2] = 2 * floor(delta_z / init_buffer_precision[2]) + 1;
	init_buffer_num[3] = 2 * floor(delta_x_deg / init_buffer_precision[3]) + 1;
	init_buffer_num[4] = 2 * floor(delta_y_deg / init_buffer_precision[4]) + 1;
	init_buffer_num[5] = 2 * floor(delta_z_deg / init_buffer_precision[5]) + 1;
	
	init_buffer_l_boundary[0] = init_buffer_var[0] - (init_buffer_num[0] - 1) / 2 *init_buffer_precision[0];
	init_buffer_l_boundary[1] = init_buffer_var[1] - (init_buffer_num[1] - 1) / 2 *init_buffer_precision[1];
	init_buffer_l_boundary[2] = init_buffer_var[2] - (init_buffer_num[2] - 1) / 2 *init_buffer_precision[2];
	init_buffer_l_boundary[3] = init_buffer_var[3] - (init_buffer_num[3] - 1) / 2 *init_buffer_precision[3];
	init_buffer_l_boundary[4] = init_buffer_var[4] - (init_buffer_num[4] - 1) / 2 *init_buffer_precision[4];
	init_buffer_l_boundary[5] = init_buffer_var[5] - (init_buffer_num[5] - 1) / 2 *init_buffer_precision[5];
	
	init_buffer_r_boundary[0] = init_buffer_var[0] + (init_buffer_num[0] - 1) / 2 *init_buffer_precision[0];
	init_buffer_r_boundary[1] = init_buffer_var[1] + (init_buffer_num[1] - 1) / 2 *init_buffer_precision[1];
	init_buffer_r_boundary[2] = init_buffer_var[2] + (init_buffer_num[2] - 1) / 2 *init_buffer_precision[2];
	init_buffer_r_boundary[3] = init_buffer_var[3] + (init_buffer_num[3] - 1) / 2 *init_buffer_precision[3];
	init_buffer_r_boundary[4] = init_buffer_var[4] + (init_buffer_num[4] - 1) / 2 *init_buffer_precision[4];
	init_buffer_r_boundary[5] = init_buffer_var[5] + (init_buffer_num[5] - 1) / 2 *init_buffer_precision[5];

	init_buffer_count_for_levels[0] = init_buffer_num[0] * init_buffer_num[1] * init_buffer_num[2] * init_buffer_num[3] * init_buffer_num[4] * init_buffer_num[5];
	init_buffer_count_for_levels[1] = init_buffer_num[1] * init_buffer_num[2] * init_buffer_num[3] * init_buffer_num[4] * init_buffer_num[5];
	init_buffer_count_for_levels[2] = init_buffer_num[2] * init_buffer_num[3] * init_buffer_num[4] * init_buffer_num[5];
	init_buffer_count_for_levels[3] = init_buffer_num[3] * init_buffer_num[4] * init_buffer_num[5];
	init_buffer_count_for_levels[4] = init_buffer_num[4] * init_buffer_num[5];
	init_buffer_count_for_levels[5] = init_buffer_num[5];

}
int DetectionMethod::getIndex(int* var) {
	return getIndex(var[0], var[1], var[2], var[3], var[4], var[5]);
}
int DetectionMethod::getIndex(int x, int y, int z, int deg_x, int deg_y, int deg_z) {
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
void DetectionMethod::creatBuffer() {
	Mat* array_buffer;
	array_buffer = new Mat[init_buffer_count_for_levels[0]];


	int* debug_array;
	debug_array = new int[init_buffer_count_for_levels[0]];
	memset(debug_array, -1, init_buffer_count_for_levels[0]* sizeof(int));
	
	
	MatchEdges creatMatchImgs(init_buffer_l_boundary, init_buffer_r_boundary,init_buffer_precision,init_buffer_count_for_levels);
	int var[6];
	Mat temp_mat;
	double count = 0;
	double count_total = init_buffer_count_for_levels[0];
	for (var[0] = init_buffer_l_boundary[0]; var[0] <= init_buffer_r_boundary[0]; var[0] += init_buffer_precision[0]) {
		for (var[1] = init_buffer_l_boundary[1]; var[1] <= init_buffer_r_boundary[1]; var[1] += init_buffer_precision[1]) {
			for (var[2] = init_buffer_l_boundary[2]; var[2] <= init_buffer_r_boundary[2]; var[2] += init_buffer_precision[2]) {
				for (var[3] = init_buffer_l_boundary[3]; var[3] <= init_buffer_r_boundary[3]; var[3] += init_buffer_precision[3]) {
					for (var[4] = init_buffer_l_boundary[4]; var[4] <= init_buffer_r_boundary[4]; var[4] += init_buffer_precision[4]) {
						for (var[5] = init_buffer_l_boundary[5]; var[5] <= init_buffer_r_boundary[5]; var[5] += init_buffer_precision[5]) {
							
							creatMatchImgs.getModelImgUchar(var, temp_mat);
							//DT变换
							creatMatchImgs.DT_L1(temp_mat,temp_mat);
							creatMatchImgs.getROI(temp_mat, temp_mat, var[0], var[1], var[2]);
							array_buffer[getIndex(var)] = temp_mat;
							count++;
							std::cout.setf(ios::fixed);
							std::cout << "Creating model DT imgs' buffer  " << setprecision(2) << count / count_total * 100 << " % " << std::endl;
/////////////////////////////////////////////////////////////////
						//for debug
							if (debug_array[getIndex(var)] == -1) {
								debug_array[getIndex(var)] = 1;
							}
							else {
								while (1) {
									std::cout << "wrong Index Input" << std::endl;
								}
							}
//////////////////////////////////////////////////////////////////////					  
							
						}
					}
				}
			}
		}
	}

	arrayMatWrite("../model/buffer.bin", array_buffer, init_buffer_count_for_levels[0]);
}
void DetectionMethod::creatBuffer_ModelPoints() {
	vector<Point2i>* points_array_buffer = new vector<Point2i>[init_buffer_count_for_levels[0]];



	int* debug_array;
	debug_array = new int[init_buffer_count_for_levels[0]];
	memset(debug_array, -1, init_buffer_count_for_levels[0] * sizeof(int));


	MatchEdges creatMatchImgs(init_buffer_l_boundary, init_buffer_r_boundary, init_buffer_precision, init_buffer_count_for_levels);
	int var[6];
	Mat temp_mat;
	double count = 0;
	double count_total = init_buffer_count_for_levels[0];
	for (var[0] = init_buffer_l_boundary[0]; var[0] <= init_buffer_r_boundary[0]; var[0] += init_buffer_precision[0]) {
		for (var[1] = init_buffer_l_boundary[1]; var[1] <= init_buffer_r_boundary[1]; var[1] += init_buffer_precision[1]) {
			for (var[2] = init_buffer_l_boundary[2]; var[2] <= init_buffer_r_boundary[2]; var[2] += init_buffer_precision[2]) {
				for (var[3] = init_buffer_l_boundary[3]; var[3] <= init_buffer_r_boundary[3]; var[3] += init_buffer_precision[3]) {
					for (var[4] = init_buffer_l_boundary[4]; var[4] <= init_buffer_r_boundary[4]; var[4] += init_buffer_precision[4]) {
						for (var[5] = init_buffer_l_boundary[5]; var[5] <= init_buffer_r_boundary[5]; var[5] += init_buffer_precision[5]) {

							creatMatchImgs.getModelImgUchar(var, temp_mat);
							cv::findNonZero(temp_mat, points_array_buffer[getIndex(var)]);
							count++;
							std::cout.setf(ios::fixed);
							std::cout << "Creating model DT imgs' buffer  " << setprecision(2) << count / count_total * 100 << " % " << std::endl;
							/////////////////////////////////////////////////////////////////
							//for debug
							if (debug_array[getIndex(var)] == -1) {
								debug_array[getIndex(var)] = 1;
							}
							else {
								while (1) {
									std::cout << "wrong Index Input" << std::endl;
								}
							}
							//////////////////////////////////////////////////////////////////////					  

						}
					}
				}
			}
		}
	}

	arrayVecOfPointsWrite("../model/buffer.bin", points_array_buffer, init_buffer_count_for_levels[0]);
}
void DetectionMethod::readBuffer_ModelPoints() {
	arrayVecOfPointsRead("../model/buffer.bin", init_buffer_count_for_levels[0]);
}
void DetectionMethod::readBuffer() {
	arrayMatRead("../model/buffer.bin", init_buffer_count_for_levels[0]);
}
void DetectionMethod::initialization() {
	cam_src = imread("../model/sample.jpg", CV_8UC1);
	cam_src_color = imread("../model/sample.jpg");

	Canny(cam_src, cam_canny_img, 50, 200);

}
void DetectionMethod::debugShowContours(int canny_index, std::vector<std::vector<Point>> *cam_contours,int cam_index, std::vector<std::vector<Point>> *model_contours,int model_index) {
	Mat back_ground = cam_src_color.clone();
	back_ground.setTo(cv::Scalar(255, 255, 255));
	drawContours(back_ground, *cam_contours, cam_index,Scalar(255,0,0));
	drawContours(back_ground, *model_contours, model_index, Scalar(0, 0, 255));
	imshow("debugShowContours", back_ground);
	imshow("debugShowModelCanny", model_ini_Canny_imgs[canny_index]);
}
void DetectionMethod::debugShowMatch(double* var) {
	// var 是位置、姿态参数，是一个大小为6的数组
	Mat back_ground = cam_src_color.clone();
	MatchEdges matchEdgesForShow(cam_canny_img,0);

	Mat model_canny_img_src;
	matchEdgesForShow.getModelImg<double>(var, model_canny_img_src);
	for (int i = 0; i < back_ground.rows; i++)
	{
		for (int j = 0; j < back_ground.cols; j++)
		{
			if (model_canny_img_src.at<float>(i, j)>0) {
				back_ground.at<Vec3b>(i, j)[0] = 200; //Blue;
				back_ground.at<Vec3b>(i, j)[1] = 100; //g;
				back_ground.at<Vec3b>(i, j)[2] = 0; //r;
			}
		}
	}
	imshow("debugShowMatchImgs_FINAL", back_ground);
}
void DetectionMethod::debugShowMatch_offline_points(int* var) {
	// var 是位置、姿态参数，是一个大小为6的数组
	Mat back_ground = cam_src_color.clone();
	std::vector<Point2i> * points_vec = &model_canny_points[getIndex(var)];
	for (std::vector<Point2i>::iterator i = points_vec->begin(); i < points_vec->end(); i++)
	{
			
				back_ground.at<Vec3b>(i->y, i->x)[0] = 200; //Blue;
				back_ground.at<Vec3b>(i->y, i->x)[1] = 100; //g;
				back_ground.at<Vec3b>(i->y, i->x)[2] = 0; //r;

	}
	
	imshow("debugShowMatchImgs", back_ground);
}
void DetectionMethod::drawPoints(Mat &img, std::vector<Point2f> points, const Scalar& color) {
	int i;
	for (i = 0; i < points.size(); i++) {
		circle(img, points[i], 5, color);
	}
}
/*
void DetectionMethod::DT_solve_with_powell(double * output_best) {
	//output_best应为大小为6的数组
	//高精度定位：边缘图像角点匹配
	
	//需先调用至少一种粗定位函数确保匹配模板和待匹配图像像素位置有重合
	MatchSolver matchSolver(cam_canny_img);
	matchSolver.setIniVar(pos_estimated[0], pos_estimated[1], pos_estimated[2], rotate_estimated[0], rotate_estimated[1], rotate_estimated[2]);
	matchSolver.solve(output_best);
		
	
    //微调位置，姿态使partial Hausdorff distance(系数0.6)最小（或者模板匹配），采用LM非线性最优化算法
	cout << "Best Position is: x:" << output_best[0] << ", y:" << output_best[1] << ", z:"<< output_best[2] << endl;
	cout << "Best Rotation is: x:" << output_best[3] << ", y:" << output_best[4] << ", z:" << output_best[5] << endl;

}
*/
/*
void DetectionMethod::DT_solve_with_PSO(double * output_best) {
	PSO psoSolver(cam_canny_img, pos_estimated, quat_estimated, 30, 6);
	double score_best;
	psoSolver.doPSO(output_best, score_best);

	cout << "Best Position is: x:" << output_best[0] << ", y:" << output_best[1] << ", z:" << output_best[2] << endl;
	cout << "Best Rotation is: x:" << output_best[3] << ", y:" << output_best[4] << ", z:" << output_best[5] << endl;
	cout << "Final score is : " << score_best << endl;
}
*/
void DetectionMethod::DT_solve_with_DE(double * output_best) {
	DE_OnlineSolver DE_solver(init_buffer_var, init_buffer_precision, init_buffer_num, init_buffer_l_boundary, init_buffer_r_boundary, init_buffer_count_for_levels);
	double score_best;
	DE_solver.solve();
	std::vector<int> vars_valide; std::vector<int> vars_non_valide;
	for (int i = 0; i < VARS_COUNT; i++) {
		if (init_buffer_num[i] > 1) vars_valide.push_back(i);
		else vars_non_valide.push_back(i);
	}
	for (int i = 0; i < vars_valide.size(); i++) {
		output_best[vars_valide[i]] = init_buffer_var[vars_valide[i]] + (*(DE_solver.best)->vars())[vars_valide[i]] * init_buffer_precision[vars_valide[i]];
	}
	for (int i = 0; i < vars_non_valide.size(); i++) {
		output_best[vars_non_valide[i]] = init_buffer_var[vars_non_valide[i]];
	}

	score_best = DE_solver.best->cost();
	cout << "Best Position of offlineSolver is: x:" << output_best[0] << ", y:" << output_best[1] << ", z:" << output_best[2] << endl;
	cout << "Best Rotation of offlineSolver is: x:" << output_best[3] << ", y:" << output_best[4] << ", z:" << output_best[5] << endl;
	cout << "Final score is : " << score_best << endl;
}
/*
void DetectionMethod::DT_solve_with_DE_offline(double * output_best) {
	DE_Offline_Solver deOfflineSolver( init_buffer_var,  init_buffer_precision,  init_buffer_num, init_buffer_l_boundary, init_buffer_r_boundary, model_DT_imgs, init_buffer_count_for_levels);
	double score_best;
	deOfflineSolver.solve();
	std::vector<int> vars_valide; std::vector<int> vars_non_valide;
	for (int i = 0; i < VARS_COUNT; i++) {
		if (init_buffer_num[i] > 1) vars_valide.push_back(i);
		else vars_non_valide.push_back(i);
	}
	for (int i = 0; i < vars_valide.size(); i++) {
		output_best[vars_valide[i]] = init_buffer_var[vars_valide[i]] + (*(deOfflineSolver.best)->vars())[vars_valide[i]]* init_buffer_precision[vars_valide[i]];
	}
	for (int i = 0; i < vars_non_valide.size(); i++) {
		output_best[vars_non_valide[i]] = init_buffer_var[vars_non_valide[i]];
	}
	
	score_best = deOfflineSolver.best->cost();
	cout << "Best Position of offlineSolver is: x:" << output_best[0] << ", y:" << output_best[1] << ", z:" << output_best[2] << endl;
	cout << "Best Rotation of offlineSolver is: x:" << output_best[3] << ", y:" << output_best[4] << ", z:" << output_best[5] << endl;
	cout << "Final score is : " << score_best << endl;
}
*/
void DetectionMethod::DT_solve_with_DE_offline_modelCanny_camDT(double * output_best) {
	DE_Offline_Solver_modelCanny_camDT deOfflineSolver(init_buffer_var, init_buffer_precision, init_buffer_num, init_buffer_l_boundary, init_buffer_r_boundary, model_canny_points, init_buffer_count_for_levels);
	double score_best;
	deOfflineSolver.solve();
	std::vector<int> vars_valide; std::vector<int> vars_non_valide;
	for (int i = 0; i < VARS_COUNT; i++) {
		if (init_buffer_num[i] > 1) vars_valide.push_back(i);
		else vars_non_valide.push_back(i);
	}
	for (int i = 0; i < vars_valide.size(); i++) {
		output_best[vars_valide[i]] = init_buffer_var[vars_valide[i]] + (*(deOfflineSolver.best)->vars())[vars_valide[i]] * init_buffer_precision[vars_valide[i]];
	}
	for (int i = 0; i < vars_non_valide.size(); i++) {
		output_best[vars_non_valide[i]] = init_buffer_var[vars_non_valide[i]];
	}

	score_best = deOfflineSolver.best->cost();
	cout << "Best Position of offlineSolver is: x:" << output_best[0] << ", y:" << output_best[1] << ", z:" << output_best[2] << endl;
	cout << "Best Rotation of offlineSolver is: x:" << output_best[3] << ", y:" << output_best[4] << ", z:" << output_best[5] << endl;
	cout << "Final score is : " << score_best << endl;
}
