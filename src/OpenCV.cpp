#include "openCV.h"
cv::Mat readSrcImg = cv::Mat::zeros(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC1); //CV_8UC3);//the raw img got from the buffer of OpenGL;
cv::Mat readSrcImgROI = cv::Mat::zeros(ROI_HEIGHT, ROI_WIDTH, CV_8UC1); //CV_8UC3);//the raw img of ROI got from the buffer of OpenGL;

DiscreteInfo discrete_info; //全局变量
std::vector<cv::Mat> model_offline_DT_imgs;//全局变量
std::vector<std::vector<cv::Point2i>> model_offline_canny_points;//全局变量
cv::Mat cam_canny_img;
std::vector<Point2i> cam_canny_points;//全局变量
Mat cam_DT;//全局变量
std::vector<double> cache_match;//全局变量
Mat cam_img_src,cam_img_color_src;
int option;
int discrete_option;
boost::mutex terminal_mutex;
ContinuousInfo continuous_info;
void creatSample() {
	//OpenGL 生成的图上下颠倒，y,x_degree,z_degree应取相反数
	rotate_degree_set[0] = -21;
	rotate_degree_set[1] =18;
	rotate_degree_set[2] = 5;
	pos_model_set[0] = 5.5;
	pos_model_set[1] = 20.5;
	pos_model_set[2] = -701;
	//quat_set = glm::quat(glm::vec3(glm::radians(rotate_degree_set[0]), glm::radians(rotate_degree_set[1]), glm::radians(rotate_degree_set[2])));
	SetEvent(nextModelEvent);
	WaitForSingleObject(sentModelEvent, INFINITE);
	namedWindow("creatSample", WINDOW_NORMAL);
	imshow("creatSample", readSrcImg);
	//cv::flip(readSrcImg, readSrcImg, 0);
	waitKey();
	imwrite("../model/sample.jpg", readSrcImg);
	imwrite("../model/sample.bmp", readSrcImg);
	SetEvent(nextModelEvent);
}
void test_camera() {
	VideoCapture cap(1);

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
	creatSample();
	test_camera();
	int input_option = DISK_IMG_INPUT;
	if (input_option == DAHENG_CAMERA_INPUT) {
		DaHengCamera da_heng_cam;
		da_heng_cam.captureCamera();
		//Canny(cam_img_src, cam_canny_img, 50, 200);
		imshow("camera_img_src", cam_img_src);
		waitKey();
	}
	else if (input_option == DISK_IMG_INPUT) {
		cam_img_src = imread("../model/sample.bmp", CV_8UC1);
		cam_img_color_src = imread("../model/sample.bmp", CV_8UC3);
		cam_canny_img = cam_img_src;
	}


	discrete_info.setInitValue(5, 20, -700, -20, 16, 4);
	discrete_info.setPrecision(1, 1, 1, 1, 1, 1);
	discrete_info.setBoundary(20, 20, 20, 4, 1, 1);

	continuous_info.setInitValue(5, 20, -700, -20, 16, 4);
	continuous_info.setBoundary(20, 20, 20, 5, 5, 5);
	discrete_option = CONTINUOUS_MATCH;
	option = MODEL_DT_CAM_CANNY_ONLINE_ROI;
	
	// Option could be MODEL_CANNY_CAM_DT_ONLINE, MODEL_CANNY_CAM_DT_OFFLINE, MODEL_DT_CAM_CANNY_ONLINE, or MODEL_DT_CAM_CANNY_ONLINE_ROI.
	//MODEL_DT_CAM_CANNY_OFFLINE is not supported (Using too much RAM space)
	if (option != MODEL_CANNY_CAM_DT_ONLINE && option != MODEL_CANNY_CAM_DT_OFFLINE && option != MODEL_DT_CAM_CANNY_ONLINE && option != MODEL_DT_CAM_CANNY_ONLINE_ROI) {
		cout << "error: DE_option input is not valide" << endl;
	}



	DetectionMethod pos_detector;
	pos_detector.initialization();
	


	//pos_detector.creatBuffer_ModelPoints();
	//pos_detector.readBuffer_ModelPoints();

	double output_best[6];
	//pos_detector.DT_solve_with_DE(output_best, MODEL_CANNY_CAM_DT_ONLINE);
	pos_detector.DT_solve_with_DE(output_best);
	//可视化
	pos_detector.debugShowMatch(output_best);


	waitKey();

	
	return 0;
}
