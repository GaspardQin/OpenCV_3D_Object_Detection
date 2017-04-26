#pragma once
#ifndef _thread_H_
#define _thread_H_

#include "windows.h"
#include <iostream>
extern HANDLE sentEvent;
extern HANDLE readEvent;

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
// 包含着色器加载库
#include "shader.h"
// 包含相机控制辅助类
#include "camera.h"
// 包含纹理加载类
#include "texture.h"
// 加载模型的类
#include "model.h"
extern GLfloat deltaTime; // 当前帧和上一帧的时间差
extern GLfloat lastFrame; // 上一帧时间
extern Camera camera;
extern GLfloat rotate_degree[3];
const int WINDOW_WIDTH = 800, WINDOW_HEIGHT = 600;
extern cv::Mat readSrcImg;
#endif