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
// ����GLFW��
#include <GLFW/glfw3.h>
// ����SOIL��
#include <SOIL.h>
// ����GLM��
#include <GLM/glm.hpp>
#include <GLM/gtc/matrix_transform.hpp>
#include <GLM/gtc/type_ptr.hpp>
// ������ɫ�����ؿ�
#include "shader.h"
// ����������Ƹ�����
#include "camera.h"
// �������������
#include "texture.h"
// ����ģ�͵���
#include "model.h"
extern GLfloat deltaTime; // ��ǰ֡����һ֡��ʱ���
extern GLfloat lastFrame; // ��һ֡ʱ��
extern Camera camera;
extern GLfloat rotate_degree[3];
const int WINDOW_WIDTH = 800, WINDOW_HEIGHT = 600;
extern cv::Mat readSrcImg;
#endif