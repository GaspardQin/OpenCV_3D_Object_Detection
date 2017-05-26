#include "openCV.h"
cv::Mat readSrcImg = cv::Mat::zeros(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3);//the raw img got from the screenshot of OpenGL;

void creatSample() {
	//OpenGL ���ɵ�ͼ���µߵ���y,x_degree,z_degreeӦȡ�෴��
	rotate_degree_set[0] = -20;
	rotate_degree_set[1] =16;
	rotate_degree_set[2] = 4;
	pos_model_set[0] = 5;
	pos_model_set[1] = 21;
	pos_model_set[2] = -178;
	quat_set = glm::quat(glm::vec3(glm::radians(rotate_degree_set[0]), glm::radians(rotate_degree_set[1]), glm::radians(rotate_degree_set[2])));
	SetEvent(readModelEvent);
	WaitForSingleObject(sentModelEvent, INFINITE);
	imshow("creatSample", readSrcImg);
	//cv::flip(readSrcImg, readSrcImg, 0);
	waitKey();
	imwrite("../model/sample.jpg", readSrcImg);
	SetEvent(readModelEvent);
}

DWORD WINAPI cvModelThreadFun(LPVOID lpParmeter) {
	creatSample();
	DetectionMethod pos_detector;
	pos_detector.initialization();//��ȡsample.jpg��Ϊcam�õ���ͼ��
	
	//debug �ֶ�����ֶ�λλ�ã����Ծ���λ����
	glm::quat quat_est = glm::quat(glm::vec3(glm::radians(-18.0), glm::radians(13.0), glm::radians(5.0)));//YXZ
	//glm::vec3 debug_angle = glm::degrees(glm::eulerAngles(quat_est));
	pos_detector.quat_estimated[0] = quat_est[0];
	pos_detector.quat_estimated[1] = quat_est[1];
	pos_detector.quat_estimated[2] = quat_est[2];
	pos_detector.pos_estimated[0] = 12;
	pos_detector.pos_estimated[1] = 17;
	pos_detector.pos_estimated[2] = -176;

	
	//����λ

	double output_best[6];
	//pos_detector.DT_solve_with_powell(output_best);
	pos_detector.DT_solve_with_DE(output_best);//�����ǵ�߾��ȶ�λ

	//���ӻ�
	pos_detector.debugShowMatch(output_best);
	waitKey();
	
	return 0;
}