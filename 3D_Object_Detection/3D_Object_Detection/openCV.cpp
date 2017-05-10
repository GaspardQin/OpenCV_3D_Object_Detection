#include "openCV.h"
cv::Mat readSrcImg = cv::Mat::zeros(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3);//the raw img got from the screenshot of OpenGL;

void creatSample() {
	rotate_degree_set[0] = 20;
	rotate_degree_set[1] = 10;
	rotate_degree_set[2] = 5;
	pos_model_set[0] = -50;
	pos_model_set[1] = 20;
	pos_model_set[2] = -178;
	SetEvent(readModelEvent);
	WaitForSingleObject(sentModelEvent, INFINITE);
	imshow("creatSample", readSrcImg);
	cv::flip(readSrcImg, readSrcImg, 0);
	waitKey();
	imwrite("./model/sample.jpg", readSrcImg);
	SetEvent(readModelEvent);
}

DWORD WINAPI cvModelThreadFun(LPVOID lpParmeter) {
	creatSample();
	DetectionMethod pos_detector;
	pos_detector.initialization();//��ȡsample.jpg��Ϊcam�õ���ͼ��
	
	//debug �ֶ�����ֶ�λλ�ã����Ծ���λ����
	pos_detector.rotate_degree_estimated[0] = 16;
	pos_detector.rotate_degree_estimated[1] = 13;
	pos_detector.rotate_degree_estimated[2] = 6;
	pos_detector.pos_estimated[0] = -30;
	pos_detector.pos_estimated[1] = 10;
	pos_detector.pos_estimated[2] = -160;

	
	//����λ

	double output_best[6];
	pos_detector.shi_TomasiDetection(output_best);//�����ǵ�߾��ȶ�λ

	//���ӻ�
	pos_detector.debugShowMatch(output_best);
	waitKey();
	
	return 0;
}
