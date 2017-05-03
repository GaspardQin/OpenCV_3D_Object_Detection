#include "openCV.h"
#include <opencv2\xfeatures2d\nonfree.hpp>
#include <opencv2\xfeatures2d.hpp>
#include "opencv2/reg/mapaffine.hpp"
#include "opencv2/reg/mapshift.hpp"
#include "opencv2/reg/mapprojec.hpp"
#include "opencv2/reg/mappergradshift.hpp"
#include "opencv2/reg/mappergradeuclid.hpp"
#include "opencv2/reg/mappergradsimilar.hpp"
#include "opencv2/reg/mappergradaffine.hpp"
#include "opencv2/reg/mappergradproj.hpp"
#include "opencv2/reg/mapperpyramid.hpp"
using namespace cv::xfeatures2d;

using namespace cv;
cv::Mat readSrcImg = cv::Mat::zeros(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3);//the raw img got from the screenshot of OpenGL;
int BlockSizeHelp = 5 * 2 + 3;
cv::Mat modelImg, modelCannyImg;
int canny_threshold_l = 615;
int canny_threshold_h = 2030;
int approxPoly_eps = 10;
void canny_trackbar(int, void*);
void contours_trackbar(int, void*);
int x_model_degree = 0;
int y_model_degree = 0;
int z_model_degree = 0;
//真实相机获取的图片的参数
Mat camSrcImg, camCannyImg,camBinImg, camCannyImgDilate;
int camBlockSize = 5;
int camApproxPolyEps = 10;
int camCannyThresholdL = 100;
int camCannyThresholdH = 1000;
int int_camera_z = int(camera_z * 10);



void camBinTrackbar(int, void*) {
	int camBlockSizeHelp = camBlockSize * 2 + 3;
	adaptiveThreshold(camSrcImg, camBinImg, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, camBlockSizeHelp, 0);
	imshow("cam_DST", camBinImg);
}

void camCannyTrackbar(int, void*)
{
	//canny边缘检测  

	Canny(camSrcImg, camCannyImg, camCannyThresholdL / 10.0, camCannyThresholdH / 10.0);
	dilate(camCannyImg, camCannyImgDilate, getStructuringElement(MORPH_RECT, Size(3, 3), Point(0, 0)));
	imshow("cam_canny", camCannyImg);
	imshow("cam_canny_dilate", camCannyImgDilate);


}
void camContoursTrackbar(int, void*) {
	std::vector<std::vector<cv::Point> > contours;
	std::vector<Vec4i> hierarchy;
	findContours(camCannyImg, contours, hierarchy, CV_RETR_CCOMP, CHAIN_APPROX_NONE);
	std::vector<std::vector<cv::Point> > contours_poly(contours.begin(), contours.end());
	int i;



	//for (i = 0; i < contours.size(); i++) {
	//	approxPolyDP(contours[i], contours_poly[i], approxPoly_eps / 100.0, true);
	//}

	Mat contoursImg = camSrcImg.clone();
	Mat contours_seleted; //select the propre square contour
	double maxArea = 0;
	int maxIndice = 0;
	int indice;
	for (i = 0; i < contours.size(); i++) {
		if (hierarchy[i][3] >= 0 && contourArea(contours[i]) > maxArea) {
			maxIndice = i;
			maxArea = contourArea(contours[i]);
		}
	}
	drawContours(contoursImg, contours, maxIndice, Scalar(0, 255, 0), 3);
	imshow("cam_contours", contoursImg);
}

void openGLTrackbar(int, void*) {
	rotate_degree[0] = x_model_degree - 180;
	rotate_degree[1] = y_model_degree - 180;
	rotate_degree[2] = z_model_degree - 180;
	camera_z = int_camera_z / 10;
	cv::flip(readSrcImg, readSrcImg, 0);
	SetEvent(readModelEvent);
}
DWORD WINAPI cvModelThreadFun(LPVOID lpParmeter) {
	SetEvent(readModelEvent);
	//processing the real camera's img

	camSrcImg = imread("./model/input.jpg", CV_8UC1);
	namedWindow("cam_DST", 1);
	namedWindow("cam_contours", 1);
	namedWindow("cam_canny", 1);
	namedWindow("GL_rotate", 1);
	//createTrackbar("cam_block_size_track", "cam_DST", &camBlockSize, 20, camBinTrackbar);
	createTrackbar("cam_eps_track", "cam_contours", &camApproxPolyEps, 200, camContoursTrackbar);
	createTrackbar("cam_threshold_l", "cam_canny", &camCannyThresholdL, 1000, camCannyTrackbar);
	createTrackbar("cam_threshold_h", "cam_canny", &camCannyThresholdH, 3000, camCannyTrackbar);
	createTrackbar("GL_rotate_x", "GL_rotate", &x_model_degree, 360, openGLTrackbar);
	createTrackbar("GL_rotate_y", "GL_rotate", &y_model_degree, 360, openGLTrackbar);
	createTrackbar("GL_rotate_z", "GL_rotate", &z_model_degree, 360, openGLTrackbar);
	createTrackbar("GL_z", "GL_rotate", &int_camera_z, int(camera_z*10), openGLTrackbar);
	//waitKey(0);






	//End of processing the real camera's img.
	
	while (1) {
		WaitForSingleObject(sentModelEvent,INFINITE);
		std::cout << "openCV processing" << std::endl;
		
		cv::flip(readSrcImg, readSrcImg, 0); //up side down the raw img obtein from OpenGL.
		//threshold(readSrcImg, modelImg, 130, 255, CV_THRESH_BINARY);
		cv::imshow("pic", readSrcImg);
		
		
		namedWindow("DST", 1);
		namedWindow("contours", 1);
		namedWindow("canny", 1);

		createTrackbar("eps_track", "contours", &approxPoly_eps, 200, contours_trackbar);
		createTrackbar("threshold_l", "canny", &canny_threshold_l, 1000, canny_trackbar);
		createTrackbar("threshold_h", "canny", &canny_threshold_h, 3000, canny_trackbar);

		std::cout << "openCV processing" << std::endl;
		
		
		waitKey();



		SetEvent(readModelEvent);

	}
	return 0;
}

void canny_trackbar(int, void*)
{
	//canny边缘检测  

	Canny(readSrcImg, modelCannyImg, canny_threshold_l / 10.0, canny_threshold_h / 10.0, 3);
	//dilate(modelImg, modelCannyImg, getStructuringElement(MORPH_RECT, Size(3, 3), Point(0, 0)));
	imshow("canny", modelCannyImg);
	//imshow("canny_dilate", modelCannyImg);

}
	

void contours_trackbar(int, void*) {
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