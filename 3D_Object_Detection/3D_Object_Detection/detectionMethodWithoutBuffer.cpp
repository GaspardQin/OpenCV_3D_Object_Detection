
#include "detectionMethodWithoutBuffer.h"

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
	cam_src = imread("./model/sample.jpg", CV_8UC1);
	cam_src_color = imread("./model/sample.jpg");

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
	MatchEdges matchEdgesForShow(&cam_canny_img);
	std::vector<Point2f> model_corners;
	matchEdgesForShow.modelCornerDect(var, model_corners);
	drawPoints(back_ground, matchEdgesForShow.cam_corners, Scalar(255, 0, 0));
	drawPoints(back_ground, model_corners, Scalar(0, 255, 0));
	imshow("debugShowMatchPoints", back_ground);
	
	Mat back_ground2 = cam_src.clone();
	Mat model_canny_img_src;
	matchEdgesForShow.getModelImg(var, model_canny_img_src);
	for (int i = 0; i < back_ground2.rows; i++)
	{
		for (int j = 0; j < back_ground2.cols; j++)
		{
			if (model_canny_img_src.at<uchar>(i, j)>0) {
				back_ground2.at<Vec3d>(i, j)[0] += 100; //Blue;
			}
		}
	}
	imshow("debugShowMatchImgs", back_ground2);
}

void DetectionMethod::drawPoints(Mat &img, std::vector<Point2f> points, const Scalar& color) {
	int i;
	for (i = 0; i < points.size(); i++) {
		circle(img, points[i], 5, color);
	}
}
void DetectionMethod::shi_TomasiDetection(double * output_best) {
	//output_best应为大小为6的数组
	//高精度定位：边缘图像角点匹配
	
	//需先调用至少一种粗定位函数确保匹配模板和待匹配图像像素位置有重合
	MatchSolver matchSolver(&cam_canny_img);
	matchSolver.setIniVar(pos_estimated[0], pos_estimated[1], pos_estimated[2], rotate_degree_estimated[0], rotate_degree_estimated[1], rotate_degree_estimated[2]);
	matchSolver.solve(output_best);
		
	
    //微调位置，姿态使partial Hausdorff distance(系数0.6)最小（或者模板匹配），采用LM非线性最优化算法
	cout << "Best Position is: x:" << output_best[0] << ", y:" << output_best[1] << ", z:"<< output_best[2] << endl;
	cout << "Best Rotation is: x:" << output_best[3] << ", y:" << output_best[4] << ", z:" << output_best[5] << endl;

}
