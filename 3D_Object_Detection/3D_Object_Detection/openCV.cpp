#include "openCV.h"
DWORD WINAPI cvModelThreadFun(LPVOID lpParmeter) {
	PosDetection pos_detector(100,5,5,5,-20,20,-20,20,-20,20);//�ֶ�λ��ת����Ϊ5�ȣ���ת��ΧΪ����20��
	pos_detector.initialization();

	pos_detector.huCoarseDetection();//���Ե�����ֶ�λ
	
	
	//visualization part
	MatchPSO visualizer(&(pos_detector.cam_canny_img), pos_detector.deg_estimated, pos_detector.pixel_pos_estimated, pos_detector.scale_ratio_estimated);
	Mat CoarseEstimation, shi_TomasiEstimation; 
	double var_0[6] = { 0 };
	visualizer.getModelImg(var_0, CoarseEstimation);
	imshow("CoarseEstimation", CoarseEstimation);
	


	pos_detector.shi_TomasiDetection();//�����ǵ��о��ȶ�λ

	//visualization part
	visualizer.getModelImg(pos_detector.var_best, shi_TomasiEstimation);
	imshow("shi_TomasiEstimation", shi_TomasiEstimation);
	imshow("cam_canny", pos_detector.cam_canny_img);
	
	return 0;
}