
#include "detectionMethodWithoutBuffer.h"

int iteral_count = 0;

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
	model_offline_canny_points.resize(array_size);
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
		model_offline_canny_points[count] = vec_point_temp;
		vec_point_temp.~vector();
		count++;
		cout.setf(ios::fixed);
		std::cout << "Reading... " << double(count)/ double(array_size)*100<<" %" << endl;
		__int64 debug1 = *(__int64*)(((char*)&(fs.tellg())) + 8);
	}
	if (count != array_size) {
		std::cout << "Error: array_size is not correct! " << std::endl;
	}
	
}
void DetectionMethod::creatBuffer_ModelPoints() {
	vector<Point2i>* points_array_buffer = new vector<Point2i>[discrete_info.count_for_levels[0]];



	int* debug_array;
	debug_array = new int[discrete_info.count_for_levels[0]];
	memset(debug_array, -1, discrete_info.count_for_levels[0] * sizeof(int));


	MatchEdges creatMatchImgs;
	int var[6];
	Mat temp_mat;
	double count = 0;
	double count_total = discrete_info.count_for_levels[0];
	for (var[0] = discrete_info.l_boundary[0]; var[0] <= discrete_info.r_boundary[0]; var[0] += discrete_info.precision[0]) {
		for (var[1] = discrete_info.l_boundary[1]; var[1] <= discrete_info.r_boundary[1]; var[1] += discrete_info.precision[1]) {
			for (var[2] = discrete_info.l_boundary[2]; var[2] <= discrete_info.r_boundary[2]; var[2] += discrete_info.precision[2]) {
				for (var[3] = discrete_info.l_boundary[3]; var[3] <= discrete_info.r_boundary[3]; var[3] += discrete_info.precision[3]) {
					for (var[4] = discrete_info.l_boundary[4]; var[4] <= discrete_info.r_boundary[4]; var[4] += discrete_info.precision[4]) {
						for (var[5] = discrete_info.l_boundary[5]; var[5] <= discrete_info.r_boundary[5]; var[5] += discrete_info.precision[5]) {


							creatMatchImgs.getModelImgUchar(var, temp_mat);
							cv::findNonZero(temp_mat, points_array_buffer[discrete_info.getIndex(var)]);
							count++;
							std::cout.setf(ios::fixed);
							std::cout << "Creating model DT imgs' buffer  " << setprecision(2) << count / count_total * 100 << " % " << std::endl;
							/////////////////////////////////////////////////////////////////
							//for debug
							if (debug_array[discrete_info.getIndex(var)] == -1) {
								debug_array[discrete_info.getIndex(var)] = 1;
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

	arrayVecOfPointsWrite("../model/buffer.bin", points_array_buffer, discrete_info.count_for_levels[0]);
}
void DetectionMethod::readBuffer_ModelPoints() {
	arrayVecOfPointsRead("../model/buffer.bin", discrete_info.count_for_levels[0]);
}

void DetectionMethod::initialization() {
	Mat temp;
	bitwise_not(cam_canny_img, temp);
	distanceTransform(temp, cam_DT, CV_DIST_L2, 3, CV_32FC1);
	cache_match.resize(discrete_info.count_for_levels[0]);
	cache_match.assign(discrete_info.count_for_levels[0], -1);
}

void DetectionMethod::debugShowMatch(double* var) {
	// var 是位置、姿态参数，是一个大小为6的数组
	Mat back_ground = cam_img_src.clone();
	MatchEdges matchEdgesForShow;

	Mat model_canny_img_src;
	matchEdgesForShow.getModelImg<double>(var, model_canny_img_src);
	for (int i = 0; i < back_ground.rows; i++)
	{
		for (int j = 0; j < back_ground.cols; j++)
		{
			if (model_canny_img_src.at<float>(i, j)>0) {
				back_ground.at<uchar>(i, j) = 200; //Blue;
			//	back_ground.at<Vec3b>(i, j)[0] = 200; //Blue;
			//	back_ground.at<Vec3b>(i, j)[1] = 100; //g;
			//	back_ground.at<Vec3b>(i, j)[2] = 0; //r;
			}
		}
	}
	imshow("debugShowMatchImgs_FINAL", back_ground);
}
void DetectionMethod::debugShowMatch_offline_points(int* var) {
	// var 是位置、姿态参数，是一个大小为6的数组
	Mat back_ground = cam_img_src.clone();
	std::vector<Point2i> * points_vec = &model_offline_canny_points[discrete_info.getIndex(var)];
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

void DetectionMethod::DT_solve_with_DE(double * output_best) {
	DE_OnlineSolver DE_solver;
	double score_best;
	DE_solver.solve();
	std::vector<int> vars_valide; std::vector<int> vars_non_valide;
	for (int i = 0; i < VARS_COUNT; i++) {
		if (discrete_info.num[i] > 1) vars_valide.push_back(i);
		else vars_non_valide.push_back(i);
	}
	for (int i = 0; i < vars_valide.size(); i++) {
		output_best[vars_valide[i]] = discrete_info.init_var[vars_valide[i]] + (*(DE_solver.best)->vars())[vars_valide[i]] * discrete_info.precision[vars_valide[i]];
	}
	for (int i = 0; i < vars_non_valide.size(); i++) {
		output_best[vars_non_valide[i]] = discrete_info.init_var[vars_non_valide[i]];
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
	DE_Offline_Solver_modelCanny_camDT deOfflineSolver;
	double score_best;
	deOfflineSolver.solve();
	std::vector<int> vars_valide; std::vector<int> vars_non_valide;
	for (int i = 0; i < VARS_COUNT; i++) {
		if (discrete_info.num[i] > 1) vars_valide.push_back(i);
		else vars_non_valide.push_back(i);
	}
	for (int i = 0; i < vars_valide.size(); i++) {
		output_best[vars_valide[i]] = discrete_info.init_var[vars_valide[i]] + (*(deOfflineSolver.best)->vars())[vars_valide[i]] * discrete_info.precision[vars_valide[i]];
	}
	for (int i = 0; i < vars_non_valide.size(); i++) {
		output_best[vars_non_valide[i]] = discrete_info.init_var[vars_non_valide[i]];
	}

	score_best = deOfflineSolver.best->cost();
	cout << "Best Position of offlineSolver is: x:" << output_best[0] << ", y:" << output_best[1] << ", z:" << output_best[2] << endl;
	cout << "Best Rotation of offlineSolver is: x:" << output_best[3] << ", y:" << output_best[4] << ", z:" << output_best[5] << endl;
	cout << "Final score is : " << score_best << endl;
}
