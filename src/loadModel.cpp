// 引入GLEW库 定义静态链接
//#define GLEW_STATIC
#include "loadModel.h"
GLfloat deltaTime = 0.0f; // 当前帧和上一帧的时间差
GLfloat lastFrame = 0.0f; // 上一帧时间
GLfloat camera_z = 0.0f;
GLfloat pos_model_set[3] = { 0.0f, 0.0f, -100.0f };
Camera camera = Camera(glm::vec3(0.0f, 0.0f, 0.0f));
float rotate_degree_set[3] = { 0.0f };
glm::quat quat_set = glm::quat(glm::vec3(0,0,0));
bool firstMouseMove = true;
GLfloat lastX = WINDOW_WIDTH / 2.0f, lastY = WINDOW_HEIGHT / 2.0f;
//glm::vec3 vec_scale = glm::vec3(1.f, 1.f, 1.f);
glm::mat4 projection;
glm::mat4 view;
glm::mat4 M_model;
void getGLROIrect(double x, double y, double z, int* lower_left_corner){
	//确定OpenGL左下角位置（对应opencv左上角）
	int column_left_pixel = round(WINDOW_WIDTH / 2 + x / (CCD_WIDTH / 2 / FOCAL_DISTANCE*(-z))*WINDOW_WIDTH - ROI_WIDTH / 2);//z为负数
	int row_top_pixel = round(WINDOW_HEIGHT / 2 - y / (CCD_WIDTH / 2 / FOCAL_DISTANCE*(-z) / WINDOW_WIDTH*WINDOW_HEIGHT)*WINDOW_HEIGHT - ROI_HEIGHT / 2);//y上下颠倒

	//防止超出边界
	if (column_left_pixel < 0) column_left_pixel = 0;
	if (row_top_pixel < 0) row_top_pixel = 0;
	lower_left_corner[0] = column_left_pixel;
	lower_left_corner[1] = row_top_pixel; //OpenGL的左下角对应OpenCV的左上角
	//lower_left_corner[2] = column_right_pixel;
	//lower_left_corner[3] = row_bottom_pixel;
}
DWORD WINAPI glThreadFun(LPVOID lpParmeter)
{
	
	if (!glfwInit())	// 初始化glfw库
	{
		std::cout << "Error::GLFW could not initialize GLFW!" << std::endl;
		return 0;
	}

	// 开启OpenGL 3.3 core profile
	std::cout << "Start OpenGL core profile version 3.3" << std::endl;
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	//glfwWindowHint(GLFW_SAMPLES, 4);
	// 创建窗口
	GLFWwindow* window = glfwCreateWindow(1, 1,
		"Loading model with AssImp", NULL, NULL);
	glfwHideWindow(window);
	if (!window)
	{
		std::cout << "Error::GLFW could not create winddow!" << std::endl;
		glfwTerminate();
		std::system("pause");
		return 0;
	}
	// 创建的窗口的context指定为当前context
	glfwMakeContextCurrent(window);

	// 注册窗口键盘事件回调函数
	glfwSetKeyCallback(window, key_callback);
	// 注册鼠标事件回调函数
	//glfwSetCursorPosCallback(window, mouse_move_callback);
	// 注册鼠标滚轮事件回调函数
	glfwSetScrollCallback(window, mouse_scroll_callback);
	// 鼠标捕获 停留在程序内
	//glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// 初始化GLEW 获取OpenGL函数
	glewExperimental = GL_TRUE; // 让glew获取所有拓展函数
	GLenum status = glewInit();
	if (status != GLEW_OK)
	{
		std::cout << "Error::GLEW glew version:" << glewGetString(GLEW_VERSION) 
			<< " error string:" << glewGetErrorString(status) << std::endl;
		glfwTerminate();
		std::system("pause");
		return 0;
	}



	//Somewhere at initialization
	GLuint fbo, fbo_render,fbo_texture; //fbo, 
	glGenFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	//std::cout << glGetError() << std::endl;

	glGenTextures(1, &fbo_texture);
	glBindTexture(GL_TEXTURE_2D, fbo_texture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, WINDOW_WIDTH, WINDOW_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL); //末尾的NULL表示我们只预分配空间，而不实际加载纹理
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_texture, 0);



	//glGenRenderbuffers(1, &fbo_render);
	//std::cout << glGetError() << std::endl;
	//glBindRenderbuffer(GL_RENDERBUFFER, fbo_render);
	//std::cout << glGetError() << std::endl;
	//glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB, WINDOW_WIDTH, WINDOW_HEIGHT);
	//std::cout << glGetError() << std::endl;
	//glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, fbo_render);
	//std::cout << glGetError() << std::endl;
	//glBindRenderbuffer(GL_RENDERBUFFER, 0);
	//std::cout << glGetError() << std::endl;
	GLuint fbo_depth;
	glGenRenderbuffers(1, &fbo_depth);
	glBindRenderbuffer(GL_RENDERBUFFER, fbo_depth);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, WINDOW_WIDTH, WINDOW_HEIGHT);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, fbo_depth);
	glBindRenderbuffer(GL_RENDERBUFFER, 0);
	//Before drawing
	//glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
	//std::cout << glGetError() << std::endl;


	// 设置视口参数
	glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
	//std::cout << glGetError() << std::endl;
	//Section1 加载模型数据 为了方便更换模型 我们从文件读取模型文件路径
	Model objModel;
	std::ifstream modelPath("../src/modelPath.txt");
	if (!modelPath)
	{
		std::cerr << "Error::could not read model path file." << std::endl;
		glfwTerminate();
		std::system("pause");
		return 0;
	}
	std::string modelFilePath;
	std::getline(modelPath, modelFilePath);
	if (modelFilePath.empty())
	{
		std::cerr << "Error::model path empty." << std::endl;
		glfwTerminate();
		std::system("pause");
		return 0;
	}
	if (!objModel.loadModel(modelFilePath))
	{
		glfwTerminate();
		std::system("pause");
		return 0;
	}
	// Section2 准备着色器程序
	Shader shader_silhouette("../src/model.vertex", "../src/model.frag","../src/model.geometry");
	Shader shader_object("../src/model.vertex", "../src/model_object.frag", "../src/model_object.geometry"); //为了隐藏掉后面重叠的线框
	//Shader shader("../src/model.vertex", "../src/model.geometry");
	glEnable(GL_DEPTH_TEST);
	
	//glEnable(GL_MULTISAMPLE);
	glEnable(GL_POLYGON_OFFSET_FILL);
	glEnable(GL_POLYGON_OFFSET_LINE);
	glEnable(GL_POLYGON_OFFSET_POINT);
	glEnable(GL_CULL_FACE);
	
	projection = glm::perspective(GLfloat(glm::atan(CCD_WIDTH / 2.0 / FOCAL_DISTANCE)),
		(GLfloat)(WINDOW_WIDTH / WINDOW_HEIGHT), 20.0f, 1000.0f); // 投影矩阵
	view = camera.getViewMatrix(); // 视变换矩阵

	//******************* 
	//从OpenGL读取图片的时候会上下颠倒，因此在渲染的时候即颠倒图片可避免使用cv::flip()函数，该函数开销过大
	projection = glm::scale(projection, glm::vec3(1.0, -1.0, 1.0));//投影矩阵上下颠倒
	glFrontFace(GL_CW); //投影矩阵上下颠倒后，需要改正前后两面的判断方法
	//*******************
	
	
	glm::mat4 model;
	glDepthFunc(GL_LEQUAL);
	while (!glfwWindowShouldClose(window))
	{

		
		WaitForSingleObject(readModelEvent, INFINITE);
		
		//GLfloat currentFrame = (GLfloat)glfwGetTime();
	//	deltaTime = currentFrame - lastFrame;
		//if (deltaTime < 1/20) continue;
	//	lastFrame = currentFrame;
		glfwPollEvents(); // 处理例如鼠标 键盘等事件

		print_model_info();//print the model info;
		// 清除颜色缓冲区 重置为指定颜色
		glClearColor(0.f, 0.0f, 0.f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		model = glm::mat4(1.0);
		model = glm::translate(model, glm::vec3(pos_model_set[0], pos_model_set[1], pos_model_set[2])); // 再调整位置
		rotate_model(rotate_degree_set, model); //先旋转
												//	M_model = model;
												//model = glm::scale(model,vec_scale); // 适当缩小模型
		std::cout << glGetError() << std::endl;



		glPolygonOffset(1.f, 0.f); //解决z-fighting 问题，object部分绘制的时候加上一定的向后的偏移



		shader_object.use();


		glUniformMatrix4fv(glGetUniformLocation(shader_object.programId, "projection"),
			1, GL_FALSE, glm::value_ptr(projection));
		glUniformMatrix4fv(glGetUniformLocation(shader_object.programId, "view"),
			1, GL_FALSE, glm::value_ptr(view));
		glUniformMatrix4fv(glGetUniformLocation(shader_object.programId, "model"),
			1, GL_FALSE, glm::value_ptr(model));
		objModel.offscreenDraw(shader_object, fbo); // 绘制物体
		//std::cout << glGetError() << std::endl;

		glPolygonOffset(0.0f, 0.f); //对于边框绘制部分，将偏移量置为0
		shader_silhouette.use();

		glUniformMatrix4fv(glGetUniformLocation(shader_silhouette.programId, "projection"),
			1, GL_FALSE, glm::value_ptr(projection));
		glUniformMatrix4fv(glGetUniformLocation(shader_silhouette.programId, "view"),
			1, GL_FALSE, glm::value_ptr(view));
		glUniformMatrix4fv(glGetUniformLocation(shader_silhouette.programId, "model"),
			1, GL_FALSE, glm::value_ptr(model));
		//std::cout << glGetError() << std::endl;
		objModel.offscreenDraw(shader_silhouette, fbo); // 绘制物体
		

		
		glUseProgram(0);

		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		
		glReadBuffer(GL_COLOR_ATTACHMENT0);//指定glReadPixels读取GL_COLOR_ATTACHMENT0内容

		if (option == MODEL_DT_CAM_CANNY_ONLINE_ROI) {
			//直接读取ROI部分
			int lower_left_corner[2];
			getGLROIrect(pos_model_set[0], pos_model_set[1], pos_model_set[2], lower_left_corner);
			glReadPixels(lower_left_corner[0], lower_left_corner[1], ROI_WIDTH, ROI_HEIGHT, GL_GREEN, GL_UNSIGNED_BYTE, readSrcImgROI.data); //从下往上读取，因此需要反转
			
			//debug use
			//glReadPixels(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, GL_GREEN, GL_UNSIGNED_BYTE, readSrcImg.data); //从下往上读取，因此需要反转
		}
		else glReadPixels(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, GL_GREEN, GL_UNSIGNED_BYTE, readSrcImg.data); //从下往上读取，因此需要反转
		//主要消耗用于在显存与内存中的传输数据，受限于PCIE的速度
		//因此使用共享内存的intel 核心显卡 可以显著降低此步骤消耗

		//cv::flip(readSrcImg, readSrcImg, 0); //时间消耗过高，采用更改projection 矩阵代替
		//std::cout << glGetError() << std::endl;
		if (glGetError() != 0) {
			while (1) {
				std::cout << "OpenGL Error" << std::endl;
			}
		}


		ResetEvent(readModelEvent);
		SetEvent(sentModelEvent);
		
		
	}
	// 释放资源
	
	//glDeleteFramebuffers(1, &fbo);
	glDeleteRenderbuffers(1, &fbo);


	glfwTerminate();
	return 0;
}
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	switch (key)
	{
	case GLFW_KEY_W:camera.handleKeyPress(FORWARD, deltaTime);
		break;
	case GLFW_KEY_S:camera.handleKeyPress(BACKWARD, deltaTime);
		break;
	case GLFW_KEY_A:camera.handleKeyPress(LEFT, deltaTime);
		break;
	case GLFW_KEY_D:camera.handleKeyPress(RIGHT, deltaTime);
		break;
	case GLFW_KEY_Q:camera.handleKeyPress(UP, deltaTime);
		break;
	case GLFW_KEY_Z:camera.handleKeyPress(DOWN, deltaTime);
		break;
	case GLFW_KEY_UP: add_rotate_degree(0, 1, 0);
		break;
	case GLFW_KEY_LEFT: add_rotate_degree(1, 0, 0);
		break;
	case GLFW_KEY_RIGHT: add_rotate_degree(-1, 0, 0);
		break;
	case GLFW_KEY_DOWN: add_rotate_degree(0, -1, 0);
		break;
	case GLFW_KEY_LEFT_BRACKET: add_rotate_degree(0, 0, 1);
		break;
	case GLFW_KEY_RIGHT_BRACKET: add_rotate_degree(0, 0, -1);
		break;

	default:
		break;
	}
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
	{
		glfwSetWindowShouldClose(window, GL_TRUE); // 关闭窗口
	}
}
void mouse_scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	camera.handleMouseScroll(yoffset);
}
void mouse_move_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (firstMouseMove) // 首次鼠标移动
	{
		lastX = xpos;
		lastY = ypos;
		firstMouseMove = false;
	}

	GLfloat xoffset = xpos - lastX;
	GLfloat yoffset = lastY - ypos;

	lastX = xpos;
	lastY = ypos;

	camera.handleMouseMove(xoffset, yoffset);
}

void rotate_model(glm::quat quat_set, glm::mat4& mat_rotate) {
	glm::mat4 only_rotate;
	only_rotate = glm:: mat4_cast(quat_set);
	//only_rotate = glm::eulerAngleYXZ(glm::radians(rotate_degree_set[1]), glm::radians(rotate_degree_set[0]), glm::radians(rotate_degree_set[2]));//yawPitchRoll顺序
	mat_rotate = mat_rotate*only_rotate;



}

void rotate_model(float rotate_degree_set[], glm::mat4& mat_rotate) {
	glm::mat4 only_rotate;
	only_rotate = glm::eulerAngleYXZ(glm::radians(rotate_degree_set[1]), glm::radians(rotate_degree_set[0]), glm::radians(rotate_degree_set[2]));//yawPitchRoll顺序
	mat_rotate = mat_rotate*only_rotate;
	//mat_rotate = glm::rotate(mat_rotate, glm::radians(rotate_degree_set[2]), glm::vec3(0.0, 0.0, 1.0));//按照矩阵乘法，先乘的，即在左边的，后生效
	//mat_rotate = glm::rotate(mat_rotate, glm::radians(rotate_degree_set[1]), glm::vec3(0.0, 1.0, 0.0));
	//mat_rotate = glm::rotate(mat_rotate, glm::radians(rotate_degree_set[0]), glm::vec3(1.0, 0.0, 0.0));


}

void set_rotate_degree(GLfloat x_degree, GLfloat y_degree, GLfloat z_degree) {
	rotate_degree_set[0] = x_degree;
	rotate_degree_set[1] = y_degree;
	rotate_degree_set[2] = z_degree;
}
GLfloat* get_rotate_degree() {
	GLfloat* rd = rotate_degree_set;
	return rd;
}
void print_rotate_degree() {
	std::cout << "x_degree  " << rotate_degree_set[0] << "  y_degree  " << rotate_degree_set[1] << "  z_degree  " << rotate_degree_set[2] << std::endl;
}
void add_rotate_degree(GLfloat x_add, GLfloat y_add, GLfloat z_add) {
	rotate_degree_set[0] = rotate_degree_set[0] +x_add;
	rotate_degree_set[1] = rotate_degree_set[1] +y_add;
	rotate_degree_set[2] = rotate_degree_set[2] +z_add;
	
}
void print_model_info() {
//	glm::vec3 debug_angle = glm::degrees(glm::eulerAngles(quat_set));

	std::cout << "model's info : " << std::endl;
	std::cout << "	pos: x " << pos_model_set[0] << "  y  " << pos_model_set[1] << "  z  " << pos_model_set[2] << std::endl;
	std::cout << "	rotate: x" << rotate_degree_set[0] << "  y  " << rotate_degree_set[1] << " z  " << rotate_degree_set[2] << std::endl;


}
void display_axis() {
	GLuint displayList = glGenLists(1);  //请求显示列表名称

	glNewList(displayList, GL_COMPILE);   //创建显示列表

	glBegin(GL_LINES);
	glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(10, 0, 0);
	glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, 10, 0);
	glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 10);
	glEnd();

	glEndList();
	glCallList(displayList);
}