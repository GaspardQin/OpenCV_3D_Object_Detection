#pragma once
#ifndef _thread_H_
#define _thread_H_

#include "windows.h"
#include <iostream>
//extern HANDLE sentImgEvent;
extern HANDLE sentModelEvent;
extern HANDLE readModelEvent;//全局变量应在相应cpp下先声明，再在thread_variables.h中声明extern
//extern HANDLE readImgEvent;

#include <opencv2\opencv.hpp>
#include <opencv2\core\opengl.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>

#include <GL/glew.h>
//#include <GL/freeglut.h>
// 引入GLFW库
#include <GLFW/glfw3.h>
// 引入SOIL库
#include <SOIL.h>
// 引入GLM库
#include <GLM/glm.hpp>
#include <GLM/gtc/matrix_transform.hpp>
#include <GLM/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp> 
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
// 包含着色器加载库
#include "shader.h"
// 包含相机控制辅助类
#include "camera.h"
// 包含纹理加载类
#include "texture.h"
// 加载模型的类
#include "model.h"
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/shared_array.hpp>

#define WINDOW_WIDTH 1920.0
#define WINDOW_HEIGHT 1080.0 //pixel
#define ROI_WIDTH 400.0
#define ROI_HEIGHT 400.0  //pixel
#define FOCAL_DISTANCE 16.0 //mm
#define CCD_WIDTH 13.0 //mm

class DiscreteInfo {
public:
	std::vector<int> init_var;
	std::vector<int> delta;
	std::vector<int> l_boundary;
	std::vector<int> r_boundary;
	std::vector<int> precision;
	std::vector<int> num;
	std::vector<int> count_for_levels;
	DiscreteInfo() {
		init_var.resize(6);
		delta.resize(6);
		l_boundary.resize(6);
		r_boundary.resize(6);
		precision.resize(6);
		num.resize(6);
		count_for_levels.resize(6);
	}
	void setInitValue(int x_init, int y_init, int z_init, int deg_x, int deg_y, int deg_z) {
		init_var[0] = x_init;
		init_var[1] = y_init;
		init_var[2] = z_init;
		init_var[3] = deg_x;
		init_var[4] = deg_y;
		init_var[5] = deg_z;
	}
	void setPrecision(int x_precision, int y_precision, int z_precision, int x_deg_precision, int y_deg_precision, int z_deg_precision) {
		precision[0] = x_precision;
		precision[1] = y_precision;
		precision[2] = z_precision;
		precision[3] = x_deg_precision;
		precision[4] = y_deg_precision;
		precision[5] = z_deg_precision;

	}

	void setBoundary(int delta_x, int delta_y, int delta_z, int delta_x_deg, int delta_y_deg, int delta_z_deg) {
		//边界与中心值差值，上下边界关于中心值对称
		delta[0] = delta_x;
		delta[1] = delta_y;
		delta[2] = delta_z;
		delta[3] = delta_x_deg;
		delta[4] = delta_y_deg;
		delta[5] = delta_z_deg;

		num[0] = 2 * floor(delta_x / precision[0]) + 1;
		num[1] = 2 * floor(delta_y / precision[1]) + 1;
		num[2] = 2 * floor(delta_z / precision[2]) + 1;
		num[3] = 2 * floor(delta_x_deg / precision[3]) + 1;
		num[4] = 2 * floor(delta_y_deg / precision[4]) + 1;
		num[5] = 2 * floor(delta_z_deg / precision[5]) + 1;

		l_boundary[0] = init_var[0] - (num[0] - 1) / 2 * precision[0];
		l_boundary[1] = init_var[1] - (num[1] - 1) / 2 * precision[1];
		l_boundary[2] = init_var[2] - (num[2] - 1) / 2 * precision[2];
		l_boundary[3] = init_var[3] - (num[3] - 1) / 2 * precision[3];
		l_boundary[4] = init_var[4] - (num[4] - 1) / 2 * precision[4];
		l_boundary[5] = init_var[5] - (num[5] - 1) / 2 * precision[5];

		r_boundary[0] = init_var[0] + (num[0] - 1) / 2 * precision[0];
		r_boundary[1] = init_var[1] + (num[1] - 1) / 2 * precision[1];
		r_boundary[2] = init_var[2] + (num[2] - 1) / 2 * precision[2];
		r_boundary[3] = init_var[3] + (num[3] - 1) / 2 * precision[3];
		r_boundary[4] = init_var[4] + (num[4] - 1) / 2 * precision[4];
		r_boundary[5] = init_var[5] + (num[5] - 1) / 2 * precision[5];

		count_for_levels[0] = num[0] * num[1] * num[2] * num[3] * num[4] * num[5];
		count_for_levels[1] = num[1] * num[2] * num[3] * num[4] * num[5];
		count_for_levels[2] = num[2] * num[3] * num[4] * num[5];
		count_for_levels[3] = num[3] * num[4] * num[5];
		count_for_levels[4] = num[4] * num[5];
		count_for_levels[5] = num[5];

	}
	int getIndex(const int* var) {
		return getIndex(var[0], var[1], var[2], var[3], var[4], var[5]);
	}
	int getIndex(const int x,const int y,const int z,const int deg_x,const int deg_y,const int deg_z) {
		//得到array中对应的index
		int index, x_count, y_count, z_count, deg_x_count, deg_y_count, deg_z_count;
		if ((x - l_boundary[0]) % precision[0] > 0) {
			while (1) {
				std::cout << "Error: wrong Index of x !" << std::endl;
			}
		}
		if ((y - l_boundary[1]) % precision[1] > 0) {
			while (1) {
				std::cout << "Error: wrong Index of y !" << std::endl;
			}
		}
		if ((z - l_boundary[2]) % precision[2] > 0) {
			while (1) {
				std::cout << "Error: wrong Index of z !" << std::endl;
			}
		}
		if ((deg_x - l_boundary[3]) % precision[3] > 0) {
			while (1) {
				std::cout << "Error: wrong Index of deg_x !" << std::endl;
			}
		}
		if ((deg_y - l_boundary[4]) % precision[4] > 0) {
			while (1) {
				std::cout << "Error: wrong Index of deg_y !" << std::endl;
			}
		}
		if ((deg_z - l_boundary[5]) % precision[5] > 0) {
			while (1) {
				std::cout << "Error: wrong Index of deg_z !" << std::endl;
			}
		}


		x_count = (x - l_boundary[0]) / precision[0];//从0开始数
		y_count = (y - l_boundary[1]) / precision[1];
		z_count = (z - l_boundary[2]) / precision[2];
		deg_x_count = (deg_x - l_boundary[3]) / precision[3];
		deg_y_count = (deg_y - l_boundary[4]) / precision[4];
		deg_z_count = (deg_z - l_boundary[5]) / precision[5];

		index = x_count * count_for_levels[1] + y_count * count_for_levels[2] + z_count * count_for_levels[3]
			+ deg_x_count *count_for_levels[4] + deg_y_count*count_for_levels[5] + deg_z_count;
		return index;

	}
};


extern boost::mutex gl_mutex;
extern boost::shared_mutex cv_cache_mutex;

extern GLfloat deltaTime; // 当前帧和上一帧的时间差
extern GLfloat lastFrame; // 上一帧时间
extern Camera camera;
extern float rotate_degree_set[3];
extern glm::quat quat_set;

extern cv::Mat readSrcImg;
extern GLfloat camera_z;
extern GLfloat pos_model_set[3];
extern glm::mat4 projection;
extern glm::mat4 view;
extern glm::mat4 M_model;
extern int iteral_count;

extern cv::Mat cam_img_src, cam_img_color_src;
extern cv::Mat cam_canny_img;
extern std::vector<cv::Point2i> cam_canny_points;
extern cv::Mat cam_DT; // float

extern std::vector<cv::Mat> model_offline_DT_imgs;
extern std::vector<std::vector<cv::Point2i>> model_offline_canny_points;
extern DiscreteInfo discrete_info;
extern std::vector<double> cache_match; //访问需要上读写锁
#endif