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

int params_manuel[6];
int params_max[6]; //正负对称
int params_manuel_init[6];
int match_threshold_u10;
int match_threshold_Ll0;
void cannyTrackbar(int, void*)
	{
		//canny边缘检测  
		Mat canny_img;
		Canny(findCannyParams_src, canny_img, threshold_l, threshold_u);

		imshow("findCannyParams", canny_img);
	};
void findPosManuelTrackbar(int, void*) {
	Mat back_ground = cam_img_color_src.clone();
	Mat back_ground2;
	cvtColor(cam_canny_img, back_ground2, CV_GRAY2RGB);
	std::vector<Point2i> contours_points;

	//boost::mutex::scoped_lock lock(gl_mutex); //保证每个时刻只有一个线程能与OpenGL通信
	gl_mutex.lock();
	pos_model_set[0] = params_manuel[0] - params_max[0]+ params_manuel_init[0];
	pos_model_set[1] = params_manuel[1] - params_max[1] + params_manuel_init[1];
	pos_model_set[2] = params_manuel[2] - params_max[2] + params_manuel_init[2]; //z为负值
	rotate_degree_set[0] = params_manuel[3] - params_max[3] + params_manuel_init[3];
	rotate_degree_set[1] = params_manuel[4] - params_max[4] + params_manuel_init[4];
	rotate_degree_set[2] = params_manuel[5] - params_max[5] + params_manuel_init[5];

	ResetEvent(sentModelEvent);
	SetEvent(nextModelEvent);

	WaitForSingleObject(sentModelEvent, INFINITE);
	double params_array[6];
	params_array[0] = pos_model_set[0];
	params_array[1] = pos_model_set[1];
	params_array[2] = pos_model_set[2]; //z为负值
	params_array[3] = rotate_degree_set[0];
	params_array[4] = rotate_degree_set[1];
	params_array[5] = rotate_degree_set[2];

	cv::findNonZero(readSrcImg, contours_points);
	ResetEvent(readImgEvent);
	SetEvent(readImgEvent);
	gl_mutex.unlock();
	Vec3b * temp;
	Vec3b* temp2;
	for (std::vector<Point2i>::iterator i = contours_points.begin(); i < contours_points.end(); i++)
	{
		temp = &back_ground.at<Vec3b>(i->y, i->x);
		temp[0] = 200; //Blue;
		temp[1] = 100; //g;
		temp[2] = 0; //r;
		temp2 = &back_ground2.at<Vec3b>(i->y, i->x);
		temp2[0] = 200; //Blue;
		temp2[1] = 100; //g;
		temp2[2] = 0; //r;back_ground2.at<uchar>(i->y, i->x)=255;
	}

	MatchEdges matchEdgesforPrint;
	double dist = matchEdgesforPrint.MatchOnline_modelDTcamCanny_continuous(params_array, match_threshold_Ll0/10.0, match_threshold_u10/10.0);
	cout << "params input: x: " << params_array[0] << " y: " << params_array[1] << " z: " << params_array[2] << " x_deg: " << params_array[3] << " y_deg: " << params_array[4] << " z_deg: " << params_array[5] << endl;

	cout << "DT score iteral " << dist << endl;

	pyrDown(back_ground, back_ground);
	imshow("findPosManuel", back_ground);
	imshow("findPosManuel_canny", back_ground2);

}
void findCannyParams(Mat& src_) {
	findCannyParams_src = src_;
	namedWindow("findCannyParams", 1);

	createTrackbar("threshold_l", "findCannyParams", &threshold_l, 100, cannyTrackbar);
	createTrackbar("threshold_h", "findCannyParams", &threshold_u, 1000, cannyTrackbar);
	waitKey();
}	

void findPosManuel() {
	params_max[0] = 10;
	params_max[1] = 20;
	params_max[2] = 20;
	params_max[3] = 10;
	params_max[4] = 10;
	params_max[5] = 10;
	params_manuel_init[0] = -8;
	params_manuel_init[1] = 13;
	params_manuel_init[2] = -246;
	params_manuel_init[3] = -8;
	params_manuel_init[4] = 2;
	params_manuel_init[5] = -27;
	params_manuel[0] = 10;
	params_manuel[1] = 20;
	params_manuel[2] = 24;
	params_manuel[3] = 12;
	params_manuel[4] = 6;
	params_manuel[5] = 10;
	namedWindow("findPosManuel", 1);
	namedWindow("findPosManuel_canny", 1);
	createTrackbar("pos_x", "findPosManuel", &params_manuel[0], 2 * params_max[0], findPosManuelTrackbar);
	createTrackbar("pos_y", "findPosManuel", &params_manuel[1], 2 * params_max[1], findPosManuelTrackbar);
	createTrackbar("pos_z", "findPosManuel", &params_manuel[2], 2 * params_max[2], findPosManuelTrackbar);
	createTrackbar("deg_x", "findPosManuel", &params_manuel[3], 2 * params_max[3], findPosManuelTrackbar);
	createTrackbar("deg_y", "findPosManuel", &params_manuel[4], 2 * params_max[4], findPosManuelTrackbar);
	createTrackbar("deg_z", "findPosManuel", &params_manuel[5], 2 * params_max[5], findPosManuelTrackbar);
	createTrackbar("L", "findPosManuel", &match_threshold_Ll0, 10, findPosManuelTrackbar);
	createTrackbar("U", "findPosManuel", &match_threshold_u10, 10, findPosManuelTrackbar);

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
		cam_img_src=imread("../model/cam_photos/sample.bmp", IMREAD_GRAYSCALE);
		cam_img_color_src = imread("../model/cam_photos/sample.bmp", IMREAD_COLOR);
		my_cam_calibrate.calibrate(cam_img_src, cam_img_src);
		my_cam_calibrate.calibrate(cam_img_color_src, cam_img_color_src);
		//findCannyParams(cam_img_src);
		//cam_img_src = cam_img_src_(Range(1, WINDOW_WIDTH), Range(0, WINDOW_HEIGHT-1));
		Canny(cam_img_src, cam_canny_img, 88, 30, 3);
	}
	

	discrete_info.setInitValue(5, 20, -700, -20, 16, 4);
	discrete_info.setPrecision(1, 1, 1, 1, 1, 1);
	discrete_info.setBoundary(10, 10, 20, 4, 1, 1);

	continuous_info.setInitValue(-8, 13, -242, -6, -3, -27);
	continuous_info.setBoundary(5, 5, 5, 5,5, 5);
	//discrete_option = CONTINUOUS_MATCH;
	discrete_option = CONTINUOUS_MATCH;
	option = MODEL_DT_CAM_CANNY_ONLINE;
	//option = MODEL_DT_CAM_CANNY_ONLINE_ROI;
	
	// Option could be MODEL_CANNY_CAM_DT_ONLINE, MODEL_CANNY_CAM_DT_OFFLINE, MODEL_DT_CAM_CANNY_ONLINE, or MODEL_DT_CAM_CANNY_ONLINE_ROI.
	//MODEL_DT_CAM_CANNY_OFFLINE is not supported (Using too much RAM space)
	if (option != MODEL_CANNY_CAM_DT_ONLINE && option != MODEL_CANNY_CAM_DT_OFFLINE && option != MODEL_DT_CAM_CANNY_ONLINE && option != MODEL_DT_CAM_CANNY_ONLINE_ROI) {
		cout << "error: DE_option input is not valide" << endl;
	}

	//namedWindow("debugShowMatchImgs", 1);

	DetectionMethod pos_detector;
	pos_detector.initialization();
	
	//findPosManuel();

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
