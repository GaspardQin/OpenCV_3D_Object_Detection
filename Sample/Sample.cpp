// Sample.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#pragma comment(lib, "ObjectDetection.lib")
using namespace Object_Detection;
int main()
{
	
	ObjectDetection object_detection;
	object_detection.setWindowSize(1920, 1080);
	object_detection.setROISize(1000, 700);
	object_detection.setCCDWidth(5.709);
	object_detection.setFocalDistance(8.0);
	object_detection.setDEoptions(4, 30, 1000, 50, 0.4, 0.6);
	object_detection.setDToptions(0.0, 0.6);
	object_detection.setDiskImgPath(string("../model/sample.bmp"));
	object_detection.setImgSourceOption(DISK_IMG_INPUT);
	object_detection.setEdgeCompareMethod(MODEL_DT_CAM_CANNY_ONLINE);
	object_detection.setPrintOptions(DETAIL_PRINT);
	object_detection.createSampleImg(0, 0, -1000,0,0,0);
	object_detection.setInitialValue(5, 20, -700, -20, 16, 4);
	object_detection.setDiscretePrecision(5, 5, 5, 2, 1, 1);
	object_detection.setDiscreteBoundary(10, 10, 10, 4, 1, 1);//必须最后调用boundary
	
	object_detection.run();
    return 0;
}

