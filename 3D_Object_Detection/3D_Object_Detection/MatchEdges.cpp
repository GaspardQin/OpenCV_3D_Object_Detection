
#include "MatchEdges.h"
using namespace std;
using namespace cv;
void MatchEdges::camCornerDect() {
	goodFeaturesToTrack(*cam_img, cam_corners, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k); //δѡ�����Ȥ����  
}
void MatchEdges::modelCornerDect(double *var, vector<Point2f> &model_corners) {
	Mat model_canny_img;
	getModelImg(var, model_canny_img);
	goodFeaturesToTrack(model_canny_img, model_corners, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k); //δѡ�����Ȥ����  
}
void MatchEdges::getModelImg(const double *var, Mat& model_canny_img) const {
	pos_model_set[0] = var[0];
	pos_model_set[1] = var[1];
	pos_model_set[2] = var[2]; //zΪ��ֵ
	rotate_degree_set[0] = var[3];
	rotate_degree_set[1] = var[4];
	rotate_degree_set[1] = var[5];

	SetEvent(readModelEvent);
	WaitForSingleObject(sentModelEvent, INFINITE);
	cv::flip(readSrcImg, readSrcImg, 0);
	Mat model_canny_img_pre;
	Canny(readSrcImg, model_canny_img_pre, 50, 200);
}

double MatchEdges::hausdorffDistance(const double *var) const {
	//���Ż���������
	Mat model_canny_img;
	vector<Point2f> model_corners;
	getModelImg(var, model_canny_img);
	goodFeaturesToTrack(model_canny_img, model_corners, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k); //δѡ�����Ȥ����  

	return (hausdorff_ptr->computeDistance(cam_corners, model_corners));
	//΢��λ�ã���̬ʹpartial Hausdorff distance(ϵ��0.6)��С������ģ��ƥ�䣩
}
