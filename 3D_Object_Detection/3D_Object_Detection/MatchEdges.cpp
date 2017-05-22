
#include "MatchEdges.h"
using namespace std;
using namespace cv;
void MatchEdges::CornerDect(Mat cam_img,std::vector<Point2f>& cam_corners) const{
	goodFeaturesToTrack(cam_img, cam_corners, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k); //未选择感兴趣区域  
	// 角点位置精准化参数  
	Size winSize = Size(5, 5);
	Size zeroZone = Size(-1, -1);
	TermCriteria criteria = TermCriteria(
		CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
		100, //maxCount=40  
		0.00001);  //epsilon=0.00001  
				 /// 计算精准化后的角点位置  
	cornerSubPix(cam_img, cam_corners, winSize, zeroZone, criteria);

}
void MatchEdges::modelCornerDect(const double *var, vector<Point2f> &model_corners)const {
	Mat model_canny_img;
	double debug_var[6];
	debug_var[0] = var[0];
	debug_var[1] = var[1];
	debug_var[2] = var[2];
	debug_var[3] = var[3];
	debug_var[4] = var[4];
	debug_var[5] = var[5];
	getModelImg(var, model_canny_img);
	goodFeaturesToTrack(model_canny_img, model_corners, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k); //未选择感兴趣区域  
	Size winSize = Size(5, 5);
	Size zeroZone = Size(-1, -1);
	TermCriteria criteria = TermCriteria(
		CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
		100, //maxCount=40  
		0.00001);  //epsilon=0.00001  
				 /// 计算精准化后的角点位置  
	cornerSubPix(model_canny_img, model_corners, winSize, zeroZone, criteria);
}
void MatchEdges::getModelImg(const double *var, Mat& model_canny_img) const {
	pos_model_set[0] = var[0];
	pos_model_set[1] = var[1];
	pos_model_set[2] = var[2]; //z为负值
	//rotate_degree_set[0] = var[3];
	//rotate_degree_set[1] = var[4];
	//rotate_degree_set[2] = var[5];
	quat_set[0] = var[3];
	quat_set[1] = var[4];
	quat_set[2] = var[5];
	quat_set[3] = sqrt(1 - quat_set[0] * quat_set[0] - quat_set[1] * quat_set[1] - quat_set[2]* quat_set[2]);
	ResetEvent(sentModelEvent);
	SetEvent(readModelEvent);
	
	WaitForSingleObject(sentModelEvent, INFINITE);
	//cv::flip(readSrcImg, readSrcImg, 0);
	Mat model_canny_img_pre;
	Canny(readSrcImg, model_canny_img, 50, 200);
}

double MatchEdges::hausdorffDistance(const double *var)const {
	//待优化能量函数
	Mat model_canny_img;
	vector<Point2f> model_corners;
	getModelImg(var, model_canny_img);
	goodFeaturesToTrack(model_canny_img, model_corners, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k); //未选择感兴趣区域  
	//double dist = hausdorff_ptr->computeDistance(cam_corners, model_corners);
	Size winSize = Size(5, 5);
	Size zeroZone = Size(-1, -1);
	TermCriteria criteria = TermCriteria(
		CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
		100, //maxCount=40  
		0.00001);  //epsilon=0.00001  
				   /// 计算精准化后的角点位置  
	cornerSubPix(model_canny_img, model_corners, winSize, zeroZone, criteria);

	double dist = hausdorffDistanceManuelSum(cam_corners, model_corners, 0.4,0.8);
	return (dist);
	//微调位置，姿态使partial Hausdorff distance(系数0.6)最小（或者模板匹配）
}
double MatchEdges::hausdorffDistance(const double *var, std::vector<Point2f> &cam_corners_nearest)const {
	//待优化能量函数
	Mat model_canny_img;
	vector<Point2f> model_corners;
	getModelImg(var, model_canny_img);
	goodFeaturesToTrack(model_canny_img, model_corners, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k); //未选择感兴趣区域  
	Size winSize = Size(5, 5);
	Size zeroZone = Size(-1, -1);
	TermCriteria criteria = TermCriteria(
		CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
		100, //maxCount=40  
		0.00001);  //epsilon=0.00001  
				   /// 计算精准化后的角点位置  
	cornerSubPix(model_canny_img, model_corners, winSize, zeroZone, criteria);																																	//double dist = hausdorff_ptr->computeDistance(cam_corners, model_corners);

	
	double dist = hausdorffDistanceManuelSum(cam_corners, model_corners, cam_corners_nearest, 0.4, 0.8);
	return (dist);
	//微调位置，姿态使partial Hausdorff distance(系数0.6)最小（或者模板匹配）
}

double MatchEdges::hausdorffDistanceManuelSum(const std::vector<Point2f> &cam_corners, const std::vector<Point2f> &model_corners, double k1, double k2) const {
	vector<double> distVecModel, distVecCam;
	double min_dist = 1000, temp_dist;
	for (int i = 0; i < model_corners.size(); i++) {
		min_dist = 1000;
		for (int j = 0; j < cam_corners.size(); j++) {
			temp_dist = sqrt(pow(cam_corners[i].x - model_corners[j].x, 2) + pow(cam_corners[i].y - model_corners[j].y, 2));
			if (temp_dist < min_dist) {
				min_dist = temp_dist;
			}
		}
		distVecModel.push_back(min_dist);
	}
	for (int i = 0; i < cam_corners.size(); i++) {
		min_dist = 1000;
		for (int j = 0; j < model_corners.size(); j++) {
			temp_dist = sqrt(pow(cam_corners[i].x - model_corners[j].x, 2) + pow(cam_corners[i].y - model_corners[j].y, 2));
			if (temp_dist < min_dist) {
				min_dist = temp_dist;
			}
		}
		distVecCam.push_back(min_dist);
	}
	sort(distVecModel.begin(), distVecModel.end());
	sort(distVecCam.begin(), distVecCam.end());
	double sum = 0;
	for (int i = floor(k2*distVecModel.size()) - 1; i > floor(k1*distVecModel.size()); i--) {
		sum += distVecModel[i];
		sum += distVecCam[i];
	}
	sum /= (floor(k2*distVecModel.size()) - floor(k1*distVecModel.size()));
	sum /= 2;

	return sum;
}

double MatchEdges::hausdorffDistanceManuelSum(const std::vector<Point2f> &cam_corners, const std::vector<Point2f> &model_corners,std::vector<Point2f> &cam_corners_nearest, double k1, double k2) const {
	vector<double> distVecModel, distVecCam;
	double min_dist = 1000, temp_dist;
	int min_index = 0;
	vector<Point2f> cam_corners_copy;
	cam_corners_copy = cam_corners;
	cam_corners_nearest.clear();
	cam_corners_nearest.resize(cam_corners.size());
	for (int i = 0; i < model_corners.size(); i++) {
		min_dist = 1000; 
		for (int j = 0; j < cam_corners_copy.size(); j++) {
			temp_dist = sqrt(pow(cam_corners_copy[j].x - model_corners[i].x, 2) + pow(cam_corners_copy[j].y - model_corners[i].y, 2));
			if (temp_dist < min_dist) {
				min_dist = temp_dist;
				min_index = j;
				
			}
		}
		cam_corners_nearest[i] = cam_corners_copy[min_index];
		cam_corners_copy.erase(cam_corners_copy.begin() + min_index);
		distVecModel.push_back(min_dist);
	}
	for (int i = 0; i < cam_corners.size(); i++) {
		min_dist = 1000;
		for (int j = 0; j < model_corners.size(); j++) {
			temp_dist = sqrt(pow(cam_corners[i].x - model_corners[j].x, 2) + pow(cam_corners[i].y - model_corners[j].y, 2));
			if (temp_dist < min_dist) {
				min_dist = temp_dist;
			}
		}
		distVecCam.push_back(min_dist);
	}
	sort(distVecModel.begin(), distVecModel.end());
	sort(distVecCam.begin(), distVecCam.end());
	double sum=0;
	for (int i = floor(k2*distVecModel.size()) - 1; i > floor(k1*distVecModel.size()); i--) {
		sum += distVecModel[i];
		sum += distVecCam[i];
	}
	sum /= (floor(k2*distVecModel.size())- floor(k1*distVecModel.size()));
	sum /= 2;
	
	return sum;
}
double MatchEdges::hausdorffDistanceManuel(const std::vector<Point2f> &cam_corners, const std::vector<Point2f> &model_corners,double k) const{
	vector<double> distVecModel, distVecCam;
	double min_dist = 1000, temp_dist;
	for (int i = 0; i < model_corners.size(); i++) {
		min_dist = 1000;
		for (int j = 0; j < cam_corners.size(); j++) {
			temp_dist = sqrt(pow(cam_corners[j].x - model_corners[i].x, 2) + pow(cam_corners[j].y - model_corners[i].y, 2));
			if (temp_dist < min_dist) {
				min_dist = temp_dist;
			}
		}
		distVecModel.push_back(min_dist);
	}
	for (int i = 0; i < cam_corners.size(); i++) {
		min_dist = 1000;
		for (int j = 0; j < model_corners.size(); j++) {
			temp_dist = sqrt(pow(cam_corners[i].x - model_corners[j].x, 2) + pow(cam_corners[i].y - model_corners[j].y, 2));
			if (temp_dist < min_dist) {
				min_dist = temp_dist;
			}
		}
		distVecCam.push_back(min_dist);
	}
	sort(distVecModel.begin(), distVecModel.end());
	sort(distVecCam.begin(), distVecCam.end());

	if (distVecModel[floor(k*distVecModel.size()) - 1] > distVecCam[floor(k*distVecCam.size()) - 1])
		return distVecModel[floor(k*distVecModel.size()) - 1];
	else return distVecCam[floor(k*distVecCam.size()) - 1];
}
double MatchEdges::hausdorffDistancePur(const double *var) const {
	Mat model_canny_img;
	getModelImg(var, model_canny_img);
	vector<Point2i> model_all_points;
	getAllPointsFromCanny(model_canny_img, model_all_points);
	double dist = hausdorff_ptr->computeDistance(cam_all_points, model_all_points);

	return (dist);
}
void MatchEdges::camAllPoints() {
	getAllPointsFromCanny(cam_img, cam_all_points);

}
void MatchEdges::getAllPointsFromCanny(Mat& model_canny_img, vector<Point2i>& model_all_points) const {
	for (int i = 0; i < model_canny_img.rows; i++)
	{
		for (int j = 0; j < model_canny_img.cols; j++)
		{
			if (model_canny_img.at<uchar>(i, j)>0) {
				model_all_points.push_back(Point2i(i,j));
			}
		}
	}
}
void MatchEdges::RANSACfilter(double * var) {
	std::vector<Point2f> model_corners, cam_corners_nearest;
	modelCornerDect(var,model_corners);
	hausdorffDistance(var, cam_corners_nearest);
	Mat mask; Mat H;
	H = findHomography(model_corners, cam_corners_nearest, CV_LMEDS,3, mask);
	
	vector<Point2f> correct_match_cam,correct_match_model;
	vector<Point2f> wrong_match_model, wrong_match_cam;

	for (int i = 0; i < mask.size().height; i++) {
		if (mask.at<uchar>(i,0) == 1) {
			correct_match_cam.push_back(cam_corners_nearest[i]);	//寻找被用于RANSAC的点
			correct_match_model.push_back(model_corners[i]);
		}
		else//寻找其他点的正确匹配
		{
			wrong_match_model.push_back(model_corners[i]);
			wrong_match_cam.push_back(cam_corners_nearest[i]);
		}
	}
	
	vector<Point2f> warp_w_model;
	perspectiveTransform(wrong_match_model, warp_w_model, H);
	for (int i = 0; i < wrong_match_model.size(); i++) {
		
		for (int j = 0; j < wrong_match_cam.size(); j++) {
			if ((abs(warp_w_model[i].x - wrong_match_cam[j].x) < 2) && (abs(warp_w_model[i].y - wrong_match_cam[j].y) < 2)) {
				correct_match_cam.push_back(wrong_match_cam[j]);
				correct_match_model.push_back(wrong_match_model[i]);
				wrong_match_cam.erase(wrong_match_cam.begin() + j);
				wrong_match_model.erase(wrong_match_model.begin() + i);
			}
		}
	} 



	//可用的点对匹配完毕，重新ransac计算变换矩阵
	H = findHomography(correct_match_model, correct_match_cam, CV_LMEDS, 1, mask);
	debugShowMatch(correct_match_model, correct_match_cam);
	waitKey();
	glm::mat4 H_glm;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (i == 3 || j ==3) {
				H_glm[j][i] = 0;  //glm中 [][]操作符先取列再取行
				//Column-major versus row-major is purely a notational convention. 
				//Note that post-multiplying with column-major matrices produces the same result as pre-multiplying with row-major matrices.
			}
			else H_glm[j][i] = H.at<double>(i, j); //转换Opencv中的矩阵到GLM中的矩阵, 并转换为4*4齐次矩阵
		}
	}
	H_glm[3][3] = 1.0;
	
	//由模型变换矩阵M_model，投影矩阵Projection,视口矩阵View 得到实物（即模型趋近的最终结果）的变换矩阵M_cam
	glm::mat4 M_cam = projection*view* H_glm* glm::inverse(view) *glm::inverse(projection)*M_model;

	glm::quat Q_rotate = glm::quat_cast(M_cam);//从M_cam中提取旋转四元数
	glm::vec3 euler_angle = glm::eulerAngles(Q_rotate);//将四元数转换为欧拉角pitch as x, yaw as y, roll as z, 仍旧是yawPitchRoll的旋转顺序
	double var_d[6];
	var_d[0] = M_cam[3][0];//x
	var_d[1] = M_cam[3][1];//y
	var_d[2] = M_cam[3][2];//z
	var_d[3] = glm::degrees(euler_angle[0]);
	var_d[4] = glm::degrees(euler_angle[1]);
	var_d[5] = glm::degrees(euler_angle[2]);
	var[0] = var_d[0];
	var[1] = var_d[1];
	var[2] = var_d[2];
	var[3] = var_d[3];
	var[4] = var_d[4];
	var[5] = var_d[5];


}
double MatchEdges::RANSACfilterDistance(double * var) const {
	std::vector<Point2f> model_corners, cam_corners_nearest;
	modelCornerDect(var, model_corners);
	hausdorffDistance(var, cam_corners_nearest);
	Mat mask; Mat H;
	H = findHomography(model_corners, cam_corners_nearest, CV_RANSAC, 5, mask);

	vector<Point2f> correct_match_cam, correct_match_model;
	vector<Point2f> wrong_match_model, wrong_match_cam;

	for (int i = 0; i < mask.size().height; i++) {
		if (mask.at<uchar>(i, 0) == 1) {
			correct_match_cam.push_back(cam_corners_nearest[i]);	//寻找被用于RANSAC的点
			correct_match_model.push_back(model_corners[i]);
		}
		else//寻找其他点的正确匹配
		{
			wrong_match_model.push_back(model_corners[i]);
			wrong_match_cam.push_back(cam_corners_nearest[i]);
		}
	}

	vector<Point2f> warp_w_model;
	perspectiveTransform(wrong_match_model, warp_w_model, H);
	for (int i = 0; i < wrong_match_model.size(); i++) {

		for (int j = 0; j < wrong_match_cam.size(); j++) {
			if ((abs(warp_w_model[i].x - wrong_match_cam[j].x) < 1.5) && (abs(warp_w_model[i].y - wrong_match_cam[j].y) < 1.5)) {
				correct_match_cam.push_back(wrong_match_cam[j]);
				correct_match_model.push_back(wrong_match_model[i]);
				//wrong_match_cam.erase(wrong_match_cam.begin() + j);
				//wrong_match_model.erase(wrong_match_model.begin() + i);
			}
		}
	}

	//可用的点对匹配完毕，重新ransac计算变换矩阵
	H = findHomography(correct_match_model, correct_match_cam, CV_LMEDS, 2, mask);
	debugShowMatch(correct_match_model, correct_match_cam);
	waitKey(10);
	/*
	double max_dist = 0;
	double temp_dist;
	for (int i = 0; i < correct_match_cam.size(); i++) {
		temp_dist = sqrt((correct_match_cam[i].x - correct_match_model[i].x)*(correct_match_cam[i].x - correct_match_model[i].x) + (correct_match_cam[i].y - correct_match_model[i].y)*(correct_match_cam[i].y - correct_match_model[i].y));
		if (max_dist < temp_dist) {
			max_dist = temp_dist;
		}
	}
	return max_dist;
	*/
	/*
	double sum_dist=0;
	for (int i = 0; i < correct_match_cam.size(); i++) {
		sum_dist += sqrt((correct_match_cam[i].x - correct_match_model[i].x)*(correct_match_cam[i].x - correct_match_model[i].x) + (correct_match_cam[i].y - correct_match_model[i].y)*(correct_match_cam[i].y - correct_match_model[i].y));
	}
	return sum_dist/ correct_match_cam.size();
	*/
	double sum = 0;
	Mat Id = Mat::eye(3, 3,CV_64FC1);
	H = H - Id;
//	subtract(H, Mat::eye(3, 3,CV_32F), H);
	//H = H - Mat::eye(3, 3, CV_32F);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			sum += H.at<double>(i, j)*H.at<double>(i, j);
		}
	}
	return sum;
}
const void MatchEdges::drawPoints(Mat &img, std::vector<Point2f> points, const Scalar& color)const {
	int i;
	for (i = 0; i < points.size(); i++) {
		circle(img, points[i], 5, color);
	}
}
const void MatchEdges::debugShowMatch(std::vector<Point2f> model_corners, std::vector<Point2f> cam_corners)const {
	// var 是位置、姿态参数，是一个大小为6的数组
	Mat cam_src_color = imread("./model/sample.jpg");
	Mat back_ground = cam_src_color.clone();

	int i;
	for (i = 0; i < model_corners.size(); i++) {
		Scalar color = Scalar(int(rand() * 255 / RAND_MAX), int(rand() * 255 / RAND_MAX), int(rand() * 255 / RAND_MAX));
		circle(back_ground, model_corners[i], 5, color);
		circle(back_ground, cam_corners[i], 5, color);
	}
	//drawPoints(back_ground, model_corners, Scalar(200, 150, 50));
	//drawPoints(back_ground, cam_corners, Scalar(50, 200, 50));
	imshow("debugShowMatchPoints_RANSAC", back_ground);
	
}
double MatchEdges::hausdorffDistancePyramid2(const double *var)const {
	//待优化能量函数
	Mat model_canny_img; Mat model_canny_pyramid_2;
	vector<Point2f> model_corners_pyramid_2;
	getModelImg(var, model_canny_img);
	binaryZoomOut(model_canny_img, model_canny_pyramid_2, 0.25);
	//pyrDown(model_canny_img, model_canny_pyramid_2);
	//pyrDown(model_canny_pyramid_2, model_canny_pyramid_2);

	//CornerDect(model_canny_pyramid_2, model_corners_pyramid_2);

	double dist = hausdorffDistanceManuelSum(cam_corners_pyramid_2, model_canny_pyramid_2, 0.4, 0.8);
	return (dist);
	
}
double MatchEdges::DTmatchHelp(Mat cam_DT, Mat model_canny_img, double k_l, double k_u) const {
	//DistanceTransform Match
	vector<double> dist;
	for (int i = 0; i < model_canny_img.size().height; i++) {
		for (int j = 0; j < model_canny_img.size().width; j++) {
			if (model_canny_img.at<uchar>(i,j) > 0){
				dist.push_back(cam_DT.at<float>(i,j));
			}
		}
	}
	sort(dist.begin(), dist.end());
	double sum = 0;
	for (int i = floor(k_u*dist.size()) - 1; i > floor(k_l*dist.size()); i--) {
		sum += dist[i];
	}
	std::cout << "max distance :" << dist[floor(k_u*dist.size()) - 1] << std::endl;
	return sum;
}
void MatchEdges::DT(Mat cam_img_, Mat &cam_DT_) const {
	bitwise_not(cam_img_, cam_img_);
	distanceTransform(cam_img_,cam_DT_, CV_DIST_L1, 3, CV_32F);
}
double MatchEdges::DTmatch(double* var, double k_l, double k_u) const {
	Mat model_canny_img;
	getModelImg(var, model_canny_img);
	return DTmatchHelp(cam_DT, model_canny_img, k_l, k_u);
}
double MatchEdges::DTmatchPyramid1(double* var, double k_l, double k_u) const {
	Mat model_canny_img;
	getModelImg(var, model_canny_img);
	//pyrDown(model_canny_img, model_canny_img);
	binaryZoomOut(model_canny_img, model_canny_img, 0.5);
	return DTmatchHelp(cam_DT_pyramid_1, model_canny_img, k_l, k_u);
}
double MatchEdges::DTmatchPyramid2(double* var, double k_l, double k_u) const {
	Mat model_canny_img;
	getModelImg(var, model_canny_img);
	//pyrDown(model_canny_img, model_canny_img);
	//pyrDown(model_canny_img, model_canny_img);
	binaryZoomOut(model_canny_img, model_canny_img, 0.25);
	return DTmatchHelp(cam_DT_pyramid_2, model_canny_img, k_l, k_u);
}
void MatchEdges::binaryZoomOut(Mat input_img, Mat &output_img, double f)const {
	resize(input_img, output_img, Size(0, 0), f, f, INTER_AREA);
	threshold(output_img, output_img, 20, 255, THRESH_BINARY);
}