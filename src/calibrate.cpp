#include "calibrate.h"
void CameraCalibration::getFiles(string path, vector<string>& files)
{
	//文件句柄  
	long long  hFile = 0; //win10下使用_findnext需使用Longlong
	//文件信息  
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("*.*").c_str(), &fileinfo)) != -1)
	{
		//bool test = _findnext(hFile, &fileinfo) == 0;
		do
		{
			//如果是目录,迭代之  
			//如果不是,加入列表  
			if (fileinfo.attrib &  _A_SUBDIR)
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles(p.assign(path).append(fileinfo.name).append("/"), files);
			}
			else
				files.push_back(p.assign(path).append(fileinfo.name));
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}
void CameraCalibration::addPicture(string name) {
	filenames.push_back(name);
}
void CameraCalibration::addPicturesDir(string dir_name) {
	getFiles(dir_name,filenames);
}
void CameraCalibration::setWorldPos(float side_length, int num_x, int num_y) {
	patternSize = Size(num_x, num_y);
	vector<Point3f> wPosEachImg;

	int i, j, t;
	for (j = 0; j < num_y; j++) {
		for (i = 0; i < num_x; i++) {
			wPosEachImg.push_back(Point3f(side_length*i, side_length*j, 0.0));
		}
	}
	for (t = 0; t < filenames.size(); t++) {
		world_pos.push_back(wPosEachImg);
	}
}
void CameraCalibration::processPictures() {
	int i; Mat imgsrc;
	//srcCorners.resize(filenames.size());
	for (i = 0; i < filenames.size(); i++) {
		imgsrc = imread(filenames[i], CV_LOAD_IMAGE_GRAYSCALE);
		//cvtColor(imgsrc, imgsrc, CV_BGR2GRAY);
		//pyrDown(imgsrc, imgOrginal);
		//pyrDown(imgOrginal, imgOrginal);
		//imshow("img", imgOrginal);
		//waitKey(0);
		vector<Point2f> temp;
		bool isfound = findChessboardCorners(imgsrc, patternSize, temp, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
		if (isfound == true) {
			//寻找棋盘点
			cout << isfound << endl;
			Size winSize = Size(5, 5);
			Size zeroZone = Size(-1, -1);
			TermCriteria criteria = TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);
			cout << "found corners of the " << i << "th picture" << endl;
			cornerSubPix(imgsrc, temp, winSize, zeroZone, criteria);
			//将棋盘角点亚像素化
			srcCorners.push_back(temp);
		}

	}


	world_pos.resize(srcCorners.size());
	Size imageSize = imgsrc.size();
	calibrateCamera(world_pos, srcCorners, imageSize,
		cameraMatrix, distCoeffs,
		rvecs, tvecs, 0,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON));
	//利用角点得到矫正矩阵
	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), Mat(), imageSize, CV_32FC1, map1, map2);

}
void CameraCalibration::calibrate(Mat src, Mat& dst) {
	remap(src, dst, map1, map2, INTER_LINEAR);
}
void CameraCalibration::saveData(const string& filename)
{
	ofstream fs(filename, fstream::binary);
	Mat *data[2];
	data[0] = &map1;
	data[1] = &map2;

	for (int i = 0; i < 2; i++) {

		// Header
		int type = data[i]->type();
		int channels = data[i]->channels();
		fs.write((char*)&data[i]->rows, sizeof(int));    // rows
		fs.write((char*)&data[i]->cols, sizeof(int));    // cols
		fs.write((char*)&type, sizeof(int));        // type
		fs.write((char*)&channels, sizeof(int));    // channels

													// Data
		if (data[i]->isContinuous())
		{
			fs.write(data[i]->ptr<char>(0), (data[i]->dataend - data[i]->datastart));
		}
		else
		{
			int rowsz = CV_ELEM_SIZE(type) * data[i]->cols;
			for (int r = 0; r <data[i]->rows; ++r)
			{
				fs.write(data[i]->ptr<char>(r), rowsz);
			}
		}
	}
}
void CameraCalibration::readData(const string& filename)
{
	Mat *data[2];
	data[0] = &map1;
	data[1] = &map2;
	ifstream fs(filename, fstream::binary);
	for (int i = 0; i < 2; i++) {
		// Header
		int rows, cols, type, channels;
		fs.read((char*)&rows, sizeof(int));         // rows
		fs.read((char*)&cols, sizeof(int));         // cols
		fs.read((char*)&type, sizeof(int));         // type
		fs.read((char*)&channels, sizeof(int));     // channels

													// Data
		Mat mat(rows, cols, type);
		fs.read((char*)mat.data, CV_ELEM_SIZE(type) * rows * cols);
		*(data[i]) = mat.clone();
	}
}

/*sample************

int main() {
CameraCalibration my_calibrator;
my_calibrator.addPicture("./pictures/1.JPG");
my_calibrator.addPicture("./pictures/2.JPG");
my_calibrator.addPicture("./pictures/3.JPG");
my_calibrator.addPicture("./pictures/4.JPG");
my_calibrator.addPicture("./pictures/5.JPG");
my_calibrator.addPicture("./pictures/6.JPG");
my_calibrator.addPicture("./pictures/7.JPG");
my_calibrator.addPicture("./pictures/8.JPG");
my_calibrator.addPicture("./pictures/9.JPG");
my_calibrator.addPicture("./pictures/10.JPG");
my_calibrator.addPicture("./pictures/11.JPG");
my_calibrator.addPicture("./pictures/12.JPG");
my_calibrator.addPicture("./pictures/13.JPG");
my_calibrator.addPicture("./pictures/14.JPG");
my_calibrator.addPicture("./pictures/15.JPG");
my_calibrator.addPicture("./pictures/16.JPG");
my_calibrator.addPicture("./pictures/17.JPG");

my_calibrator.setWorldPos(3, 7, 3);
my_calibrator.processPictures();

my_calibrator.saveData("./calibrationData.xml");

Mat src = imread("./pictures/1.JPG");
pyrDown(src, src);
pyrDown(src, src);
Mat dst;
my_calibrator.calibrate(src, dst);
pyrDown(dst, dst);
namedWindow("dst");
imshow("dst", dst);
waitKey(0);

}
*/