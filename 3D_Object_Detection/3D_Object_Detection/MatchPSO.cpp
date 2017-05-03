
#include "MatchPSO.h"
using namespace std;
using namespace cv;
void MatchPSO::camCornerDect() {
	goodFeaturesToTrack(*cam_img, cam_corners_PSO, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k); //δѡ�����Ȥ����  
}

void MatchPSO::getModelImg(double *var, Mat& model_canny_img) {
	rotate_degree[0] = deg_estimated_before[0] + var[0];
	rotate_degree[1] = deg_estimated_before[1] + var[1];
	rotate_degree[2] = deg_estimated_before[2] + var[2];
	SetEvent(readModelEvent);
	WaitForSingleObject(sentModelEvent, INFINITE);
	cv::flip(readSrcImg, readSrcImg, 0);
	Mat model_canny_img_pre;
	Canny(readSrcImg, model_canny_img_pre, 500, 1000);
	Mat warp_mat(2, 3, CV_32FC1);
	warp_mat.at<double>(0, 0) = scale_ratio_estimated_before + var[5]/10;
	warp_mat.at<double>(0, 1) = 0;
	warp_mat.at<double>(1, 0) = scale_ratio_estimated_before + var[5] /10; //��������ϵ���仯��
	warp_mat.at<double>(1, 1) = 0;
	warp_mat.at<double>(0, 2) = pixel_pos_estimated_before[0] + var[3];
	warp_mat.at<double>(1, 2) = pixel_pos_estimated_before[1] + var[4];
	warpAffine(model_canny_img_pre, model_canny_img, warp_mat, model_canny_img_pre.size());
}

double MatchPSO::hausdorffCost(double *var) {
	//����Ⱥcost����
	Mat model_canny_img;
	vector<Point2f> model_corners_PSO;
	getModelImg(var, model_canny_img);
	goodFeaturesToTrack(model_canny_img, model_corners_PSO, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k); //δѡ�����Ȥ����  

	return (hausdorff_ptr->computeDistance(cam_corners_PSO, model_corners_PSO));
	//΢��λ�ã���̬��z���룬����ϵ��ʹpartial Hausdorff distance(ϵ��0.6)��С������ģ��ƥ�䣩����������Ⱥ�㷨
}
