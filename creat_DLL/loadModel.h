#pragma once
#include <iostream>
#include <vector>
#include <cstdlib>
#include "stdafx.h"
namespace Object_Detection {
	
	class glThread {
	public:
		void glThreadFun();
		void rotate_model(float rotate_degree_set[], glm::mat4& mat_rotate);
		void print_rotate_degree();
		void print_model_info();
		void getGLROIrect(double x, double y, double z, int* lower_left_corner, int& modified_window_width, int& modified_window_height);
	private:

	};

}



	
	
	



