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
Mat cam_img_src;
Mat cam_img_color_src(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3);
int option;
int discrete_option;
boost::mutex terminal_mutex;
ContinuousInfo continuous_info;
CameraCalibration my_cam_calibrate;

void creatSample() {
	//OpenGL 生成的图上下颠倒，y,x_degree,z_degree应取相反数
	rotate_degree_set[0] = -21;
	rotate_degree_set[1] =17;
	rotate_degree_set[2] = 5;
	pos_model_set[0] = 5.5;
	pos_model_set[1] = 21.5;
	pos_model_set[2] = -701;
	SetEvent(nextModelEvent);
	WaitForSingleObject(sentModelEvent, INFINITE);
	namedWindow("creatSample", WINDOW_NORMAL);
	imshow("creatSample", readSrcImg);
	
	waitKey();
	//readSrcImg.at<uchar>(500, 500) = 255;
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

void calibrate_init() {
	my_cam_calibrate.addPicturesDir("../model/calibration_photos/"); 
	my_cam_calibrate.setWorldPos(0.5, 9, 9);
	my_cam_calibrate.processPictures();
	my_cam_calibrate.saveData("../model/calibrationData.bin");
}
///////////
	int threshold_l;
	int threshold_u;
	Mat findCannyParams_src;
	void cannyTrackbar(int, void*)
	{
		//canny边缘检测  
		Mat canny_img;
		Canny(findCannyParams_src, canny_img, threshold_l, threshold_u);

		imshow("findCannyParams", canny_img);
	};
	void findCannyParams(Mat& src_) {
		findCannyParams_src = src_;
		namedWindow("findCannyParams", 1);

		createTrackbar("threshold_l", "findCannyParams", &threshold_l, 1000, cannyTrackbar);
		createTrackbar("threshold_h", "findCannyParams", &threshold_u, 1000, cannyTrackbar);
		waitKey();
	}	
//////////

DWORD WINAPI cvModelThreadFun(LPVOID lpParmeter) {
	//creatSample();
	//test_camera();
	//calibrate_init();
	my_cam_calibrate.readData("../model/calibrationData.bin");
	int input_option = DISK_IMG_INPUT;
	if (input_option == DAHENG_CAMERA_INPUT) {
		DaHengCamera da_heng_cam;
		da_heng_cam.captureCamera();
		//Canny(cam_img_src, cam_canny_img, 50, 200);
		imshow("camera_img_src", cam_img_src);
		waitKey();
	}
	else if (input_option == DISK_IMG_INPUT) {
		cam_img_src = imread("../model/cam_photos/sample.bmp", IMREAD_GRAYSCALE);
		cam_img_color_src = imread("../model/cam_photos/sample.bmp", IMREAD_COLOR);
		my_cam_calibrate.calibrate(cam_img_src, cam_img_src);
		my_cam_calibrate.calibrate(cam_img_color_src, cam_img_color_src);
		//findCannyParams(cam_img_src);
		
		Canny(cam_img_src, cam_canny_img, 88, 30, 3);
	}


	discrete_info.setInitValue(5, 20, -700, -20, 16, 4);
	discrete_info.setPrecision(1, 1, 1, 1, 1, 1);
	discrete_info.setBoundary(10, 10, 20, 4, 1, 1);

	continuous_info.setInitValue(0, 0, -600, 0, 0, 0);
	continuous_info.setBoundary(30, 30, 100, 89, 89, 89);
	//discrete_option = CONTINUOUS_MATCH;
	discrete_option = CONTINUOUS_MATCH;
	option = MODEL_DT_CAM_CANNY_ONLINE;
	//option = MODEL_DT_CAM_CANNY_ONLINE_ROI;
	
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
