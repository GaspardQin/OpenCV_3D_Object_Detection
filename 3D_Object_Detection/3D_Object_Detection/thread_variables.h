#pragma once
#ifndef _thread_H_
#define _thread_H_

#include "windows.h"
#include <iostream>
//extern HANDLE sentImgEvent;
extern HANDLE sentModelEvent;
extern HANDLE readModelEvent;//ȫ�ֱ���Ӧ����Ӧcpp��������������thread_variables.h������extern
//extern HANDLE readImgEvent;

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
#include <glm/gtc/quaternion.hpp> 
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
// ������ɫ�����ؿ�
#include "shader.h"
// ����������Ƹ�����
#include "camera.h"
// �������������
#include "texture.h"
// ����ģ�͵���
#include "model.h"
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>
extern GLfloat deltaTime; // ��ǰ֡����һ֡��ʱ���
extern GLfloat lastFrame; // ��һ֡ʱ��
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