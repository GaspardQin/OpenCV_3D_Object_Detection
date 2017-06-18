#pragma once
#include <cstdio>
#include <opencv2/opencv.hpp>
//#include "getAllFilesName.cpp"
#include <direct.h>  
#include <io.h>  
#include <iostream>  
#include <stdlib.h>  
#include <stdio.h>  
#include <string.h>  
#include <vector>
#include <Windows.h>
using namespace std;
using namespace cv;

class CameraCalibration {
	//������ͼ��̫�󣬴����л��Զ���С16����������ͼ�����غ��ʣ�����Ҫ�����ã�
private:
	vector<vector<Point3f>> world_pos;
	vector<string> filenames;
	Size patternSize;
	vector<vector<Point2f>> srcCorners;
	void getFiles(string path, vector<string>& files);
	Mat cameraMatrix, distCoeffs, map1, map2;
	vector<Mat> rvecs, tvecs;
public:

	void addPicture(string name);
	void addPicturesDir(string dir_name);
	void setWorldPos(float side_length, int num_x, int num_y);
	void processPictures();
	void calibrate(Mat src,Mat& dst);
	void saveData(const string & filename);
	void readData(const string & filename);

};

