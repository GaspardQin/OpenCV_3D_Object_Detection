
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
	matchEdgesForShow.getModelImg(var, model_canny_img_src);
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
	imshow("debugShowMatchImgs", back_ground);
}

void DetectionMethod::drawPoints(Mat &img, std::vector<Point2f> points, const Scalar& color) {
	int i;
	for (i = 0; i < points.size(); i++) {
		circle(img, points[i], 5, color);
	}
}
void DetectionMethod::DT_solve_with_powell(double * output_best) {
	//output_best应为大小为6的数组
	//高精度定位：边缘图像角点匹配
	
	//需先调用至少一种粗定位函数确保匹配模板和待匹配图像像素位置有重合
	MatchSolver matchSolver(cam_canny_img);
	matchSolver.setIniVar(pos_estimated[0], pos_estimated[1], pos_estimated[2], quat_estimated[0], quat_estimated[1], quat_estimated[2]);
	matchSolver.solve(output_best);
		
	
    //微调位置，姿态使partial Hausdorff distance(系数0.6)最小（或者模板匹配），采用LM非线性最优化算法
	cout << "Best Position is: x:" << output_best[0] << ", y:" << output_best[1] << ", z:"<< output_best[2] << endl;
	cout << "Best Rotation is: x:" << output_best[3] << ", y:" << output_best[4] << ", z:" << output_best[5] << endl;

}
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
	DEsolver deSolver(pos_estimated, quat_estimated);
	double score_best;
	deSolver.solve();
	output_best[0] = (*(deSolver.best)->vars())[0];
	output_best[1] = (*(deSolver.best)->vars())[1];
	output_best[2] = (*(deSolver.best)->vars())[2];
	output_best[3] = (*(deSolver.best)->vars())[3]/rho_quat;
	output_best[4] = (*(deSolver.best)->vars())[4]/rho_quat;
	output_best[5] = (*(deSolver.best)->vars())[5]/rho_quat;
	score_best = deSolver.best->cost();
	cout << "Best Position is: x:" << output_best[0] << ", y:" << output_best[1] << ", z:" << output_best[2] << endl;
	cout << "Best Rotation is: x:" << output_best[3] << ", y:" << output_best[4] << ", z:" << output_best[5] << endl;
	cout << "Final score is : " << score_best << endl;
}
