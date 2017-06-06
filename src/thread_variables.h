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

extern GLfloat deltaTime; // ��ǰ֡����һ֡��ʱ���
extern GLfloat lastFrame; // ��һ֡ʱ��
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