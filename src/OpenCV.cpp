#include "openCV.h"
cv::Mat readSrcImg = cv::Mat::zeros(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3); //CV_8UC3);//the raw img got from the screenshot of OpenGL;

void creatSample() {
	//OpenGL 生成的图上下颠倒，y,x_degree,z_degree应取相反数
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
void test_camera() {
	VideoCapture cap(CV_CAP_PVAPI);

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the video cam" << endl;
		return ;
	}

	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

	cout << "Frame size : " << dWidth << " x " << dHeight << endl;

	namedWindow("MyVideo", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

	while (1)
	{
		Mat frame;

		bool bSuccess = cap.read(frame); // read a new frame from video

		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		imshow("MyVideo", frame); //show the frame in "MyVideo" window

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}
	

}

DWORD WINAPI cvModelThreadFun(LPVOID lpParmeter) {
	
	Mat camera_img_src;
	test_camera();
	imshow("camera_img_src", camera_img_src);


	DetectionMethod pos_detector;
	pos_detector.initialization();//读取sample.jpg作为cam得到的图像
	
	//生成buffer，仅第一次运行需要


	pos_detector.setBufferInitValue(5, 20, -700, -20, 16, 4);
	pos_detector.setBufferPrecision(1, 1, 1, 1,1, 1);
	pos_detector.setBufferBoundary(20,20, 10, 4, 1, 1);

	//pos_detector.creatBuffer_ModelPoints();
	//pos_detector.readBuffer_ModelPoints();



	//debug 手动输入粗定位位置，调试精定位方法
//	glm::quat quat_est = glm::quat(glm::vec3(glm::radians(-18.0), glm::radians(13.0), glm::radians(5.0)));//YXZ
	//glm::vec3 debug_angle = glm::degrees(glm::eulerAngles(quat_est));
	//pos_detector.quat_estimated[0] = quat_est[0];
	//pos_detector.quat_estimated[1] = quat_est[1];
	//pos_detector.quat_estimated[2] = quat_est[2];


	//精定位

	double output_best[6];
	//pos_detector.DT_solve_with_powell(output_best);
	pos_detector.DT_solve_with_DE(output_best);
	//pos_detector.DT_solve_with_DE_offline_modelCanny_camDT(output_best);
	//可视化
	pos_detector.debugShowMatch(output_best);


	waitKey();

	
	return 0;
}
