#include "openCV.h"
cv::Mat readSrcImg = cv::Mat::zeros(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3); //CV_8UC3);//the raw img got from the screenshot of OpenGL;

void creatSample() {
	//OpenGL ���ɵ�ͼ���µߵ���y,x_degree,z_degreeӦȡ�෴��
	rotate_degree_set[0] = -20;
	rotate_degree_set[1] =16;
	rotate_degree_set[2] = 4;
	pos_model_set[0] = 5;
	pos_model_set[1] = 20;
	pos_model_set[2] = -700;
	//quat_set = glm::quat(glm::vec3(glm::radians(rotate_degree_set[0]), glm::radians(rotate_degree_set[1]), glm::radians(rotate_degree_set[2])));
	SetEvent(readModelEvent);
	WaitForSingleObject(sentModelEvent, INFINITE);
	imshow("creatSample", readSrcImg);
	//cv::flip(readSrcImg, readSrcImg, 0);
	waitKey();
	imwrite("../model/sample.jpg", readSrcImg);
	imwrite("../model/sample.bmp", readSrcImg);
	SetEvent(readModelEvent);
}


DWORD WINAPI cvModelThreadFun(LPVOID lpParmeter) {
	creatSample();
	DetectionMethod pos_detector;
	pos_detector.initialization();//��ȡsample.jpg��Ϊcam�õ���ͼ��
	
	//����buffer������һ��������Ҫ
	//pos_detector.setBufferInitValue(5, 10, -180, -10, 10, 5);
	//pos_detector.setBufferPrecision(2, 2, 2, 2, 2, 2);
	//pos_detector.setBufferBoundary(20, 20, 20, 4, 1, 1);
	//pos_detector.creatBuffer();

	pos_detector.setBufferInitValue(5, 20, -700, -20, 16, 4);
	pos_detector.setBufferPrecision(5, 5, 4, 4,2, 2);
	pos_detector.setBufferBoundary(10,10, 4, 2, 1, 1);

	pos_detector.creatBuffer_ModelPoints();
	pos_detector.readBuffer_ModelPoints();



	//debug �ֶ�����ֶ�λλ�ã����Ծ���λ����
//	glm::quat quat_est = glm::quat(glm::vec3(glm::radians(-18.0), glm::radians(13.0), glm::radians(5.0)));//YXZ
	//glm::vec3 debug_angle = glm::degrees(glm::eulerAngles(quat_est));
	//pos_detector.quat_estimated[0] = quat_est[0];
	//pos_detector.quat_estimated[1] = quat_est[1];
	//pos_detector.quat_estimated[2] = quat_est[2];
	pos_detector.rotate_estimated[0] = -18;
	pos_detector.rotate_estimated[1] = 13;
	pos_detector.rotate_estimated[2] = 5;
	pos_detector.pos_estimated[0] = 7;
	pos_detector.pos_estimated[1] = 17;
	pos_detector.pos_estimated[2] = -700;

	
	//����λ

	double output_best[6];
	//pos_detector.DT_solve_with_powell(output_best);
	//pos_detector.DT_solve_with_DE(output_best);//�����ǵ�߾��ȶ�λ
	//pos_detector.DT_solve_with_DE_offline(output_best);
	pos_detector.DT_solve_with_DE_offline_modelCanny_camDT(output_best);
	//���ӻ�
	pos_detector.debugShowMatch(output_best);
	waitKey();
	
	return 0;
}
