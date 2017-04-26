// 引入GLEW库 定义静态链接
//#define GLEW_STATIC
#include "loadModel.h"
GLfloat deltaTime = 0.0f; // 当前帧和上一帧的时间差
GLfloat lastFrame = 0.0f; // 上一帧时间
Camera camera = Camera(glm::vec3(0.0f, 1.0f, 5.0f));
GLfloat rotate_degree[3] = { 0.0f };



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
	glfwWindowHint(GLFW_SAMPLES, 4);
	// 创建窗口
	GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT,
		"Loading model with AssImp", NULL, NULL);
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
	//glfwSetScrollCallback(window, mouse_scroll_callback);
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

	// 设置视口参数
	glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
	
	//Section1 加载模型数据 为了方便更换模型 我们从文件读取模型文件路径
	Model objModel;
	std::ifstream modelPath("modelPath.txt");
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
	Shader shader("model_normal.vertex", "model_normal.frag","model_normal.geom");

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_MULTISAMPLE);
	//glEnable(GL_LINE_SMOOTH);
	
	glEnable(GL_CULL_FACE);
	//glStencilFunc(GL_NOTEQUAL, 1, -1);
	//glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
	//glLineWidth(3);
	//glPolygonMode(GL_FRONT, GL_LINE);
	// 开始游戏主循环
	while (!glfwWindowShouldClose(window))
	{
		WaitForSingleObject(readEvent,INFINITE);


		GLfloat currentFrame = (GLfloat)glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		//if (deltaTime < 1/20) continue;
		lastFrame = currentFrame;
		glfwPollEvents(); // 处理例如鼠标 键盘等事件
		//do_movement(); // 根据用户操作情况 更新相机属性

		// 清除颜色缓冲区 重置为指定颜色
		glClearColor(0.f, 0.0f, 0.f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		shader.use();

		glm::mat4 projection = glm::perspective(camera.mouse_zoom,
			(GLfloat)(WINDOW_WIDTH) / WINDOW_HEIGHT, 1.0f, 1000.0f); // 投影矩阵
		glm::mat4 view = camera.getViewMatrix(); // 视变换矩阵
		glUniformMatrix4fv(glGetUniformLocation(shader.programId, "projection"),
			1, GL_FALSE, glm::value_ptr(projection));
		glUniformMatrix4fv(glGetUniformLocation(shader.programId, "view"),
			1, GL_FALSE, glm::value_ptr(view));
		
		glm::mat4 model = glm::mat4(1.0);
		rotate_model(rotate_degree, model);
		//model = glm::translate(model, glm::vec3(0.0f, -1.55f, -1.0f)); // 适当调整位置
		model = glm::scale(model, glm::vec3(0.01f, 0.01f, 0.01f)); // 适当缩小模型
		glUniformMatrix4fv(glGetUniformLocation(shader.programId, "model"),
			1, GL_FALSE, glm::value_ptr(model));
		// 这里填写场景绘制代码
		//glHint(GL_LINE_SMOOTH, GL_NICEST);
		glLineWidth(5.0f);
		objModel.draw(shader); // 绘制物体

		glBindVertexArray(0);
		glUseProgram(0);

		//read pixels into opencv mat
		//use fast 4-byte alignment (default anyway) if possible
		//glPixelStorei(GL_PACK_ALIGNMENT, (temp.step & 3) ? 1 : 4);

		//set length of one complete row in destination data (doesn't need to equal img.cols)
		//glPixelStorei(GL_PACK_ROW_LENGTH, temp.step / temp.elemSize());
		glReadPixels(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, GL_RGB, GL_UNSIGNED_BYTE, readSrcImg.data);

		glfwSwapBuffers(window); // 交换缓存





		SetEvent(sentEvent);
		
	}
	// 释放资源
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

void rotate_model(GLfloat rotate_degree[], glm::mat4& mat_rotate) {
	mat_rotate = glm::rotate(mat_rotate, glm::radians(rotate_degree[0]), glm::vec3(1.0, 0.0, 0.0));
	mat_rotate = glm::rotate(mat_rotate, glm::radians(rotate_degree[1]), glm::vec3(0.0, 1.0, 0.0));
	mat_rotate = glm::rotate(mat_rotate, glm::radians(rotate_degree[2]), glm::vec3(0.0, 0.0, 1.0));

}
void set_rotate_degree(GLfloat x_degree, GLfloat y_degree, GLfloat z_degree) {
	rotate_degree[0] = x_degree;
	rotate_degree[1] = y_degree;
	rotate_degree[2] = z_degree;
}
GLfloat* get_rotate_degree() {
	GLfloat* rd = rotate_degree;
	return rd;
}
void print_rotate_degree() {
	std::cout << "x_degree  " << rotate_degree[0] << "  y_degree  " << rotate_degree[1] << "  z_degree  " << rotate_degree[2] << std::endl;
}
void add_rotate_degree(GLfloat x_add, GLfloat y_add, GLfloat z_add) {
	rotate_degree[0] = rotate_degree[0] +x_add;
	rotate_degree[1] = rotate_degree[1] +y_add;
	rotate_degree[2] = rotate_degree[2] +z_add;
	
}