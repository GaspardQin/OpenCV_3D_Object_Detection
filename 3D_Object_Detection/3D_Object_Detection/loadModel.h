#pragma once

#include <iostream>
#include <vector>
#include <cstdlib>
#include "thread_variables.h"




	// 键盘回调函数原型声明
	// 鼠标移动回调函数原型声明
	void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);
	void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
	void mouse_scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
	

	// 场景中移动
	//void do_movement();
	//bool creat_window();

	DWORD WINAPI glThreadFun(LPVOID lpParmeter);
	void rotate_model(GLfloat rotate_degree_set[], glm::mat4& mat_rotate);
	void set_rotate_degree(GLfloat x, GLfloat y, GLfloat z);
	GLfloat* get_rotate_degree();
	void print_rotate_degree();
	void add_rotate_degree(GLfloat x_add, GLfloat y_add, GLfloat z_add);
	void print_camera_info();
	void display_axis();
	//static loadModel* static_this; //用于规避类内callback必须是static问题,只可以有一个loadModel实例
	//void set_this() { static_this = this;}
	
	// 定义程序常量
	
	// 用于相机交互参数
	//GLfloat lastX = WINDOW_WIDTH / 2.0f, lastY = WINDOW_HEIGHT / 2.0f;
	//bool firstMouseMove = true;
	//bool keyPressedStatus[1024]; // 按键情况记录

	
	
	



