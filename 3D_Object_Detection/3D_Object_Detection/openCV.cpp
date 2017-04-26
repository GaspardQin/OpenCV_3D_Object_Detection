
#include "openCV.h"
using namespace cv;
cv::Mat readSrcImg = cv::Mat::zeros(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3);//the raw img got from the screenshot of OpenGL;
int block_size_help = 5 * 2 + 3;
cv::Mat modelImg, modelCannyImg;
int canny_threshold_l = 534;
int canny_threshold_h = 1943;
int approxPoly_eps = 10;
void canny_trackbar(int, void*);
static void contours_trackbar(int, void*);
DWORD WINAPI cvThreadFun(LPVOID lpParmeter) {

	SetEvent(readEvent); 
	while (1) {
		WaitForSingleObject(sentEvent,INFINITE);
		std::cout << "openCV processing" << std::endl;
		
		cv::flip(readSrcImg, readSrcImg, 0); //up side down the raw img obtein from OpenGL.
		threshold(readSrcImg, modelImg, 130, 255, CV_THRESH_BINARY);
		cv::imshow("pic", modelImg);
		
		
		namedWindow("DST", 1);
		namedWindow("contours", 1);
		namedWindow("canny", 1);

		createTrackbar("eps_track", "contours", &approxPoly_eps, 200, contours_trackbar);
		createTrackbar("threshold_l", "canny", &canny_threshold_l, 1000, canny_trackbar);
		createTrackbar("threshold_h", "canny", &canny_threshold_h, 3000, canny_trackbar);

		std::cout << "openCV processing" << std::endl;
		
		
		waitKey();



		SetEvent(readEvent);

	}
	return 0;
}

void canny_trackbar(int, void*)
{
	//canny±ßÔµ¼ì²â  

	Canny(modelImg, modelCannyImg, canny_threshold_l / 10.0, canny_threshold_h / 10.0);
	//dilate(modelImg, modelCannyImg, getStructuringElement(MORPH_RECT, Size(3, 3), Point(0, 0)));
	imshow("canny", modelCannyImg);
	//imshow("canny_dilate", modelCannyImg);
}

static void contours_trackbar(int, void*) {
	std::vector<std::vector<cv::Point> > contours;
	std::vector<Vec4i> hierarchy;
	findContours(modelCannyImg, contours, hierarchy, CV_RETR_CCOMP, CHAIN_APPROX_NONE);
	std::vector<std::vector<cv::Point> > contours_poly(contours.begin(), contours.end());
	int i;
	


	for (i = 0; i < contours.size(); i++) {
		approxPolyDP(contours[i], contours_poly[i], approxPoly_eps / 100.0, true);
	}

	Mat contoursImg = modelImg.clone();
	Mat contours_seleted; //select the propre square contour, which have the max area.
	double maxArea = 0;
	int maxIndice = 0;
	int indice;
	for (i = 0; i < contours_poly.size(); i++) {
		if (hierarchy[i][3] >= 0 && contourArea(contours_poly[i]) > maxArea) {
			maxIndice = i;
			maxArea = contourArea(contours_poly[i]);
		}
	}
	drawContours(readSrcImg, contours_poly, maxIndice, Scalar(255, 255, 0), 1);
	imshow("contours", readSrcImg);
}