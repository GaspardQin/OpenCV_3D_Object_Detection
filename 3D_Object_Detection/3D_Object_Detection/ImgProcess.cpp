#include "ImgProcess.h"
void ImgProcess::readFile(cv::String path) {
	srcImg = imread(path, CV_8UC1);
}
void ImgProcess::debugProcess() {
	namedWindow("DST_photo", 1);
	namedWindow("contours_photo", 1);
	namedWindow("canny_photo", 1);
	createTrackbar("block_size_track", "DST_photo", &block_size_fake, 20, binary_trackbar_proxy);
	createTrackbar("eps_track", "contours_photo", &approxPoly_eps_fake, 200, contours_trackbar_proxy);
	createTrackbar("threshold_l", "canny_photo", &threshold_l_fake, 1000, canny_trackbar_proxy);
	createTrackbar("threshold_h", "canny_photo", &threshold_h_fake, 3000, canny_trackbar_proxy);
	waitKey(0);
}

void ImgProcess::binary_trackbar(int block_size) {
	int block_size_help = block_size * 2 + 3;
	adaptiveThreshold(srcImg, binImg, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, block_size_help, 0);
	imshow("DST", binImg);
}

void ImgProcess::canny_trackbar(int threshold_l)
{
	//canny±ßÔµ¼ì²â  

	Canny(srcImg, cannyImg, threshold_l / 10.0, threshold_h / 10.0);
	dilate(cannyImg, cannyImgDilate, getStructuringElement(MORPH_RECT, Size(3, 3), Point(0, 0)));
	imshow("canny", cannyImg);
	imshow("canny_dilate", cannyImgDilate);
}

void ImgProcess::contours_trackbar(int) {
	std::vector<std::vector<cv::Point> > contours;
	std::vector<Vec4i> hierarchy;
	findContours(cannyImg, contours, hierarchy, CV_RETR_CCOMP, CHAIN_APPROX_NONE);
	std::vector<std::vector<cv::Point> > contours_poly(contours.begin(), contours.end());
	int i;



	for (i = 0; i < contours.size(); i++) {
		approxPolyDP(contours[i], contours_poly[i], approxPoly_eps / 100.0, true);
	}

	Mat contoursImg = srcImg.clone();
	Mat contours_seleted; //select the propre square contour
	double maxArea = 0;
	int maxIndice = 0;
	int indice;
	for (i = 0; i < contours_poly.size(); i++) {
		if (hierarchy[i][3] >= 0 && contourArea(contours_poly[i]) > maxArea) {
			maxIndice = i;
			maxArea = contourArea(contours_poly[i]);
		}
	}
	drawContours(contoursImg, contours_poly, maxIndice, Scalar(0, 0, 0), 1);
	imshow("contours", contoursImg);
}