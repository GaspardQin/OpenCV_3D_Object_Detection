#pragma once
#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
using namespace cv;
class ImgProcess{

private:
	int block_size=5;
	int approxPoly_eps=10;
	int threshold_l=100;
	int threshold_h=1000;
public:
	Mat srcImg,binImg,cannyImg,contours, cannyImgDilate;
	void readFile(cv::String path);//Only read the gray img.
	void readVideoFrame();
	void debugProcess(); // write configure file into storage.
	void releaseProcess();

	void canny_trackbar(int);
	void binary_trackbar(int);
	void contours_trackbar(int);

	static void canny_trackbar_proxy(int v, void *ptr)
	{
		// resolve 'this':
		ImgProcess *that = (ImgProcess*)ptr;
		that->canny_trackbar(v);
	}
	static void binary_trackbar_proxy(int v, void *ptr)
	{
		// resolve 'this':
		ImgProcess *that = (ImgProcess*)ptr;
		that->binary_trackbar(v);
	}
	static void contours_trackbar_proxy(int v, void *ptr)
	{
		// resolve 'this':
		ImgProcess *that = (ImgProcess*)ptr;
		that->contours_trackbar(v);
	}
};