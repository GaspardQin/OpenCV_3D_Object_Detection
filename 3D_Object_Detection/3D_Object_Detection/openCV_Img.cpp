#include "openCV_Img.h"
#include "ImgProcess.h"
using namespace cv;

DWORD WINAPI cvImgThreadFun(LPVOID lpParmeter) {
	SetEvent(readImgEvent);
	ImgProcess processer;
	while (1) {
		//WaitForSingleObject(sentImgEvent,INFINITE);
		processer.readFile("./model/input.jpg");
		processer.debugProcess();
		
		SetEvent(readImgEvent);
		
	}
	return 0;
}