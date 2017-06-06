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


extern boost::mutex gl_mutex;
extern boost::mutex cv_cache_mutex;

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
extern Mat camera_img_src;
#endif