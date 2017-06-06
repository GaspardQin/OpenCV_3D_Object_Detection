#pragma once

#ifndef  _DETECTION_METHOD_H_
#define _DETECTION_METHOD_H_

#include "thread_variables.h"
#include "MatchSolver.h"
#include "DE.h"
using namespace cv;
using namespace std;

class DetectionMethod {
public:
	

	void creatBuffer_ModelPoints();
	void readBuffer_ModelPoints();
	void arrayVecOfPointsWrite(const string & filename, const vector<Point2i>* points_vector_array, const int array_size);
	void arrayVecOfPointsRead(const string& filename, int array_size);
	void initialization();

	void DT_solve_with_DE(double * output_best);
	void debugShowContours(int canny_index, std::vector<std::vector<Point>> *cam_contours, int cam_index, std::vector<std::vector<Point>> *model_contours, int model_index);
	void debugShowMatch(double* var);
	void debugShowMatch_offline_points(int * var);
	void drawPoints(Mat &img,std::vector<Point2f> points, const Scalar& color);
	void DT_solve_with_DE_offline_modelCanny_camDT(double * output_best);
	DetectionMethod() {
	};
	

};
#endif // ! _DETECTION_METHOD_H_
