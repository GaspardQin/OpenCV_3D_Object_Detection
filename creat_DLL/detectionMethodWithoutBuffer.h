#pragma once

#ifndef  _DETECTION_METHOD_H_
#define _DETECTION_METHOD_H_
#include "stdafx.h"
#include "DE.h"
namespace Object_Detection {


	using namespace cv;
	using namespace std;

	class DetectionMethod {
	private:
	public:
	

		void creatBuffer_ModelPoints();
		void readBuffer_ModelPoints();
		void arrayVecOfPointsWrite(const string & filename, const vector<vector<Point2i>>& points_vector_array, const int& array_size);
		void arrayVecOfPointsRead(const string& filename, int& array_size);
		void arrayMatWrite(const string& filename, const Mat* matrices, const int array_size);
		void arrayMatRead(const string& filename, int array_size, boost::shared_array<Mat> model_DT_mats_output);
		void initialization();
		void DT_solve_with_DE(int * output_best);

		void debugShowMatch(int* var);
		void debugShowMatch_offline_points(int * var);
		void drawPoints(Mat &img, std::vector<Point2f>& points, const Scalar& color);


		DetectionMethod() {
		};
	

	};

}
#endif // ! _DETECTION_METHOD_H_
