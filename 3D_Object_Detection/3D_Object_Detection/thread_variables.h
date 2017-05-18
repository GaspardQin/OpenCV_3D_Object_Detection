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
extern GLfloat deltaTime; // 当前帧和上一帧的时间差
extern GLfloat lastFrame; // 上一帧时间
extern Camera camera;
extern GLfloat rotate_degree_set[3];
const int WINDOW_WIDTH = 800, WINDOW_HEIGHT = 600;
extern cv::Mat readSrcImg;
extern GLfloat camera_z;
extern GLfloat pos_model_set[3];
extern glm::mat4 projection;
extern glm::mat4 view;
extern glm::mat4 M_model;
#endif