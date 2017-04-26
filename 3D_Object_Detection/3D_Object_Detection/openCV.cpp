
#include "openCV.h"
using namespace cv;
cv::Mat readSrcImg = cv::Mat::zeros(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3);//the raw img got from the screenshot of OpenGL;
DWORD WINAPI cvThreadFun(LPVOID lpParmeter) {

	SetEvent(readEvent); 
	while (1) {
		
		WaitForSingleObject(sentEvent,INFINITE);
		//cv::flip(readSrcImg, readSrcImg, 0); //up side down the raw img obtein from OpenGL.

		//cv::imshow("pic", readSrcImg);
		std::cout << "openCV processing" << std::endl;
		//waitKey();

		

		SetEvent(readEvent);

	}
	return 0;
}