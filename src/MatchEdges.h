#pragma once
#include "thread_variables.h"
using namespace std;
using namespace cv;
class MatchEdges {
private:
	float* row_camDT_ptrs[int(WINDOW_HEIGHT)]; //cam_DT����ÿһ�е�һ��ֵ�ĵ�ַ��Ϊ��ȡ��at������������Ԫ�صĿ��ٷ��ʣ�



public:
	Mat cam_img_debug;

	MatchEdges() {
		for (int i = 0; i < cam_DT.rows; i++) {
			row_camDT_ptrs[i] = cam_DT.ptr<float>(i);
		}
	};
	void getModelImgUchar(const int* var) const;
	double DTmatchHelp(Mat cam_DT, Mat model_canny_img, double k_l, double k_u) const;

	double MatchOnline_modelDTcamCanny(const int* var, double k_l, double k_u) const;
	double MatchOnline_modelCannycamDT(const int * var, double k_l, double k_u) const;
	double MatchOffline_modelCannycamDT(const int* var, double k_l, double k_u) const;
	double MatchOnline_modelDTcamCannyROI(const int* var, double k_l, double k_u) const;
	void DT(Mat cam_img, Mat &cam_DT) const;
	void MatchEdges::DT_L1(Mat cam_img_, Mat &cam_DT_) const;
	
	void binaryZoomOut(Mat input_img, Mat &output_img, double f)const;
	void getROI(Mat img_input, Mat& ROI_output, double x, double y, double z)const;
	void getROIrect(double x, double y, double z, int* output_array) const;
	
};




