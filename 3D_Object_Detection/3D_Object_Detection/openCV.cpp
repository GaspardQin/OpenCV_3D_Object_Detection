#include "openCV.h"
DWORD WINAPI cvModelThreadFun(LPVOID lpParmeter) {
	PosDetection pos_detector(100,5,5,5,-20,20,-20,20,-20,20);//粗定位旋转精度为5度，旋转范围为正负20度
	pos_detector.initialization();

	pos_detector.huCoarseDetection();//外边缘轮廓粗定位
	
	
	//visualization part
	MatchPSO visualizer(&(pos_detector.cam_canny_img), pos_detector.deg_estimated, pos_detector.pixel_pos_estimated, pos_detector.scale_ratio_estimated);
	Mat CoarseEstimation, shi_TomasiEstimation; 
	double var_0[6] = { 0 };
	visualizer.getModelImg(var_0, CoarseEstimation);
	imshow("CoarseEstimation", CoarseEstimation);
	


	pos_detector.shi_TomasiDetection();//轮廓角点中精度定位

	//visualization part
	visualizer.getModelImg(pos_detector.var_best, shi_TomasiEstimation);
	imshow("shi_TomasiEstimation", shi_TomasiEstimation);
	imshow("cam_canny", pos_detector.cam_canny_img);
	
	return 0;
}